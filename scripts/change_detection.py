import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv

import matplotlib.pyplot as plt

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    # 'font.size': 18,
    # 'xtick.labelsize': 18,
    # 'ytick.labelsize': 18,
    'axes.linewidth': 1,
    'axes.labelsize': 12,
})

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import ros2_numpy
from pylgmath import se3op


class BagFileParser():

  def __init__(self, bag_file):
    try:
      self.conn = sqlite3.connect(bag_file)
    except Exception as e:
      print('Could not connect: ', e)
      raise Exception('could not connect')

    self.cursor = self.conn.cursor()

    ## create a message (id, topic, type) map
    topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
    self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
    self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of, type_of in topics_data}

  # Return messages as list of tuples [(timestamp0, message0), (timestamp1, message1), ...]
  def get_bag_messages(self, topic_name):
    topic_id = self.topic_id[topic_name]
    rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
    return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]

  def get_bag_msgs_iter(self, topic_name):
    topic_id = self.topic_id[topic_name]
    result = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id))
    while True:
      res = result.fetchone()
      if res is not None:
        yield [res[0], deserialize_message(res[1], self.topic_msg_message[topic_name])]
      else:
        break

def get_result_from_msg(msg):
  data = ros2_numpy.numpify(msg)
  num_points = data.shape[0]
  #
  query = np.empty((num_points, 3))
  query[:, 0] = data['flex11']
  query[:, 1] = data['flex12']
  query[:, 2] = data['flex13']
  #
  centroid = np.empty((num_points, 3))
  centroid[:, 0] = data['x']
  centroid[:, 1] = data['y']
  centroid[:, 2] = data['z']
  #
  normal = np.empty((num_points, 3))
  normal[:, 0] = data['normal_x']
  normal[:, 1] = data['normal_y']
  normal[:, 2] = data['normal_z']
  #
  result = np.empty((num_points, 4))
  result[:, 0] = data['flex21']  # distance (point to plane)
  result[:, 1] = data['flex22']  # roughness
  result[:, 2] = data['flex23']  # num observations
  result[:, 3] = data['flex24']  # ground truth
  #
  return query, centroid, normal, result

def load_results(data_dir, data_name):
  data_file = f"{data_dir}/{data_name}/{data_name}_0.db3"
  data_parser = BagFileParser(data_file)
  data_iter = data_parser.get_bag_msgs_iter(data_name)

  #
  filtered_results = []
  for time, msg in data_iter:
    #
    query, centroid, normal, result = get_result_from_msg(msg)
    # valid distance and roughness
    filter = np.logical_and.reduce((result[:, 0] >= 0, result[:, 1] > 0))
    # filter = (result[:, 1] > 0)
    filtered_result = result[filter]
    filtered_results.append(filtered_result)
  filtered_results = np.concatenate(filtered_results, axis=0)
  return filtered_results


def plot_precision_recall(ax, cost, ground_truth, color='b', label=None):
  precisions, recalls = [], []
  sorted_cost = np.sort(cost)
  thresholds = [sorted_cost[0]-1.0] + [sorted_cost[i] for i in range(0, len(sorted_cost), int(len(sorted_cost) / 100))] + [sorted_cost[-1]]

  for threshold in thresholds:
    prediction = cost > threshold
    TP = np.sum(np.logical_and(prediction, ground_truth))
    TN = np.sum(np.logical_and(np.logical_not(prediction), np.logical_not(ground_truth)))
    FP = np.sum(np.logical_and(prediction, np.logical_not(ground_truth)))
    FN = np.sum(np.logical_and(np.logical_not(prediction), ground_truth))
    if ((TP + FP) > 0) and ((TP + FN) > 0):
      print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{TP / (TP + FP):>10.2f}, R:{TP / (TP + FN):>10.2f}")
      precisions.append(TP / (TP + FP))
      recalls.append(TP / (TP + FN))
    else:
      print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{0:>10.2f}, R:{0:>10.2f}")
  ax.plot(recalls, precisions, color=color, label=label)


def main(data_dir):
  data_dir = osp.join(osp.normpath(data_dir), "graph/data")

  fig_title = "dome inside fake objects"
  fig, axs = plt.subplots(2, 4, figsize=(16, 12))
  fig.subplots_adjust(left=0.07, right=0.95, wspace=0.3, hspace=0.4)
  fig.suptitle(fig_title)

  ###
  filtered_results = load_results(data_dir, "change_detection_result")
  weights = np.ones(len(filtered_results)) / float(len(filtered_results))

  # roughness with a prior
  alpha_0 = 1.0 * 2  # prior number of observations
  beta_0 = 0.1 * 2   # prior beta
  alpha_n = alpha_0 + filtered_results[:, 2] / 2
  beta_n = beta_0 + np.square(filtered_results[:, 1]) * filtered_results[:, 2] / 2
  roughness = beta_n / alpha_n

  axs[0][0].hist(roughness, bins=40, log=True, color='g', weights=weights)
  axs[0][0].set_title("vertical stddev [m]")

  # vertical distance results
  axs[0][1].hist(filtered_results[:, 0], bins=40, log=True, color='r', weights=weights)
  axs[0][1].set_title("vertical distance [m]")

  # negative log prob
  df = 2 * alpha_n
  sqdists = (filtered_results[:, 0]**2) / roughness
  cost = -np.log((1 + sqdists / df)**(-(df + 1) / 2))
  axs[0][2].hist(cost, bins=40, log=True, color='b', weights=weights)
  axs[0][2].set_title("vertical -logprob")

  # precision recall curve
  plot_precision_recall(axs[0][3], cost, filtered_results[:, 3]>0.5, color='b', label="vertical -logprob")
  plot_precision_recall(axs[0][3], filtered_results[:, 0], filtered_results[:, 3]>0.5, color='r', label="vertical distance")
  axs[0][3].legend()
  axs[0][3].set_title("vertical PR curve")
  axs[0][3].set_ylabel("precision")
  axs[0][3].set_xlabel("recall")

  ## point to plane results
  filtered_results = load_results(data_dir, "change_detection_v2_result")
  weights = np.ones(len(filtered_results)) / float(len(filtered_results))


  # add a prior
  alpha_0 = 1.0 * 2  # prior number of observations
  beta_0 = 0.1 * 2   # prior beta
  alpha_n = alpha_0 + filtered_results[:, 2] / 2
  beta_n = beta_0 + np.square(filtered_results[:, 1]) * filtered_results[:, 2] / 2
  roughness = beta_n / alpha_n

  axs[1][0].hist(roughness, bins=40, log=True, color='g', weights=weights)
  axs[1][0].set_title("closest plane stddev [m]")

  # distance
  axs[1][1].hist(filtered_results[:, 0], bins=40, log=True, color='r', weights=weights)
  axs[1][1].set_title("point-to-plane distance [m]")

  # negative log prob
  df = 2 * alpha_n
  sqdists = (filtered_results[:, 0]**2) / roughness
  cost = -np.log((1 + sqdists / df)**(-(df + 1) / 2))
  axs[1][2].hist(cost, bins=40, log=True, color='b', weights=weights)
  axs[1][2].set_title("closest plane -logprob")

  # precision recall curve
  plot_precision_recall(axs[1][3], cost, filtered_results[:, 3]>0.5, color='b', label="plane -logprob")
  plot_precision_recall(axs[1][3], filtered_results[:, 0], filtered_results[:, 3]>0.5, color='r', label="plane distance")
  axs[1][3].legend()
  axs[1][3].set_title("closest plane PR curve")
  axs[1][3].set_ylabel("precision")
  axs[1][3].set_xlabel("recall")

  plt.show()

if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <path>/graph/data/<database directory>
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)