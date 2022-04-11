import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv
import matplotlib.pyplot as plt

from scipy import spatial

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
  result = np.empty((num_points, 5))
  result[:, 0] = data['flex21']  # distance (point to plane)
  result[:, 1] = data['flex22']  # roughness
  result[:, 2] = data['flex23']  # num observations
  result[:, 3] = data['flex24']  # ground truth
  result[:, 4] = data['flex14']  # normal agreements
  #
  return query, centroid, normal, result

def compute_cost(query, centroid, normal, result):
  alpha_0 = 1.0 * 2  # prior number of observations
  beta_0 = 0.1 * 2   # prior beta
  alpha_n = alpha_0 + result[:, 2] / 2
  beta_n = beta_0 + result[:, 1] * result[:, 2] / 2
  roughness = beta_n / alpha_n

  df = 2 * alpha_n
  sqdists = (result[:, 0]**2) / roughness
  cost = -np.log((1 + sqdists / df)**(-(df + 1) / 2))

  return roughness, cost


def find_neighbors(query, support_radius=0.25):
  kdtree = spatial.KDTree(query)
  neighbors = kdtree.query_ball_point(query, support_radius)
  return neighbors

def smooth_v2(threshold, query, cost, neighbors, results, support_threshold, support_variance, normal_threshold):
  query_indices = np.arange(query.shape[0])

  condition = np.logical_and(cost <= threshold, results[:, 4] < normal_threshold)
  condition = np.logical_or(cost > threshold, condition)
  label = np.where(condition, 1., 0.)
  new_label = np.zeros_like(label)

  change = query[label == 1]
  change_neighbors = neighbors[label == 1]
  change_indices = query_indices[label == 1]

  for i, inds in enumerate(change_neighbors):
    nn_point = query[inds]
    nn_dists = np.linalg.norm(nn_point - change[i], axis=1)
    nn_label = label[inds]
    supports = nn_label * np.exp(-nn_dists**2 / support_variance)
    total_support = np.sum(supports)
    new_label[change_indices[i]] = 1.0 if total_support > support_threshold else 0.0
  return new_label


def plot_precision_recall3(ax, cost, ground_truth, prediction_func, thresholds=None, color=None, label=None):
  precisions, recalls = [], []
  if thresholds is None:
    sorted_cost = np.sort(cost)
    thresholds = [sorted_cost[0]-1.0] + [sorted_cost[i] for i in range(0, len(sorted_cost), int(len(sorted_cost) / 5))] + [sorted_cost[-1]]

  for threshold in thresholds:
    prediction = prediction_func(threshold)
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
  return thresholds


def plot_precision_recall(ax, cost, ground_truth, color=None, label=None):
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
  return thresholds


def main(data_dir):
  data_dir = osp.join(osp.normpath(data_dir), "graph/data")

  fig_title = "backyard fake objects"
  fig, axs = plt.subplots(1, 1, figsize=(4.5, 4))
  fig.subplots_adjust(left=0.12, right=0.98, wspace=0.3, hspace=0.4)
  fig.suptitle(fig_title)

  ## point to plane results
  data_name = "change_detection_v2_result"
  data_file = f"{data_dir}/{data_name}/{data_name}_0.db3"
  data_parser = BagFileParser(data_file)
  data_iter = data_parser.get_bag_msgs_iter(data_name)

  #
  filtered_queries = []
  filtered_results = []
  filtered_rougnhesses = []
  filtered_costs = []
  filtered_neighbors = []
  for time, msg in data_iter:
    #
    query, centroid, normal, result = get_result_from_msg(msg)
    # valid distance and roughness
    filter = np.logical_and.reduce((result[:, 0] >= 0, result[:, 1] > 0))
    # apply filtering
    ftd_query = query[filter]
    ftd_centroid = centroid[filter]
    ftd_normal = normal[filter]
    ftd_result = result[filter]
    ftd_roughness, ftd_cost = compute_cost(ftd_query, ftd_centroid, ftd_normal, ftd_result)
    ftd_neighbors = find_neighbors(ftd_query)

    filtered_queries.append(ftd_query)
    filtered_results.append(ftd_result)
    filtered_rougnhesses.append(ftd_roughness)
    filtered_costs.append(ftd_cost)
    filtered_neighbors.append(ftd_neighbors)

  def get_label(threshold,
      queries = filtered_queries,
      costs = filtered_costs,
      neighbors = filtered_neighbors,
      results = filtered_results,
      support_threshold = 1.5,
      support_variance = 0.1,
      normal_threshold=0.0):
    labels = []
    for i in range(len(queries)):
      labels.append(smooth_v2(threshold, queries[i], costs[i], neighbors[i], results[i], support_threshold, support_variance, normal_threshold))
    return np.concatenate(labels, axis=0)

  def get_label_meta(support_threshold = 1.5,normal_threshold=0.0):
    return lambda threshold: get_label(threshold, support_threshold = support_threshold, normal_threshold=normal_threshold)

  filtered_results = np.concatenate(filtered_results, axis=0)
  filtered_rougnhesses = np.concatenate(filtered_rougnhesses, axis=0)
  filtered_costs = np.concatenate(filtered_costs, axis=0)

  # precision recall curve
  thresholds = plot_precision_recall(axs, filtered_costs, filtered_results[:, 3]>0.5, label="unfiltered")
  # downsampled_thresholds = thresholds[:1]
  # for th in thresholds[1:]:
  #   if np.abs(th - downsampled_thresholds[-1]) > 0.005:
  #     downsampled_thresholds.append(th)
  downsampled_thresholds = thresholds[-10:]
  # plot_precision_recall3(axs, filtered_costs, filtered_results[:, 3]>0.5, get_label_meta(2.5), downsampled_thresholds, label="filtered (2.5)")
  plot_precision_recall3(axs, filtered_costs, filtered_results[:, 3]>0.5, get_label_meta(0.0, 0.0), downsampled_thresholds, label="filtered (2.5)")
  axs.legend()
  axs.set_title("closest plane PR curve")
  axs.set_ylabel("precision")
  axs.set_xlabel("recall")

  plt.show()

if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <path>/graph/data/<database directory>
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)