import os
import os.path as osp
import argparse
import numpy as np
import numpy.linalg as npla
import csv
import matplotlib.pyplot as plt
from multiprocessing import Pool
from scipy import spatial

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    'font.size': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'axes.linewidth': 1,
    'axes.titlesize': 10,
    'axes.labelsize': 10,
    'legend.fontsize': 10,
})

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import ros2_numpy
from pylgmath import se3op

def setup_figure(row, col, height, width, l=0.0, r=0.0, b=0.0, t=0.0, w=0.0, h=0.0):
  tot_height = row * height
  tot_width = col * width
  # axes spacing
  tot_height = tot_height + (height * h) * (row - 1)
  tot_width = tot_width + (width * w) * (col - 1)
  # left right padding
  tot_height = tot_height / (1 - (b + t))
  tot_width = tot_width / (1 - (l + r))

  fig, axs = plt.subplots(row, col, figsize=(tot_width, tot_height))
  fig.subplots_adjust(left=l, right=1.0-r, bottom=b, top=1.0-t)
  fig.subplots_adjust(wspace=w, hspace=h)
  print(f"Figure size (width, height): {fig.get_size_inches()}")
  return fig, axs

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
  roughness = result[:, 1]
  cost = (result[:, 0]**2) / (2 * roughness) + np.log(np.sqrt(roughness))
  return roughness, cost

def compute_cost_prior(query, centroid, normal, result):
  alpha_0 = 6.0 / 2  # prior number of observations
  beta_0 = 0.01 * alpha_0  # prior beta
  # alpha_0 = 1.0 * 2  # prior number of observations
  # beta_0 = 0.1 * 2   # prior beta
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

def support_filter(threshold, query, cost, neighbors, support_threshold, support_variance):
  query_indices = np.arange(query.shape[0])

  label = np.where(cost > threshold, 1., 0.)
  new_label = np.zeros_like(label)

  change = query[label == 1.]
  change_neighbors = neighbors[label == 1.]
  change_indices = query_indices[label == 1.]
  for i, inds in enumerate(change_neighbors):
    nn_point = query[inds]
    nn_dists = np.linalg.norm(nn_point - change[i], axis=1)
    nn_label = label[inds]
    supports = nn_label * np.exp(-nn_dists**2 / (2*support_variance))
    total_support = np.sum(supports)
    new_label[change_indices[i]] = 1.0 if total_support > support_threshold else 0.0
  return new_label

def plot_precision_recall(ax, cost, ground_truth,
                          prediction_func=lambda threshold, cost: cost > threshold,
                          thresholds=None, color=None, label=None, save=None):
  if thresholds is None:
    sorted_cost = np.sort(cost)
    thresholds = [sorted_cost[0]-1.0] + [sorted_cost[i] for i in range(0, len(sorted_cost), int(len(sorted_cost) / 50))] + [sorted_cost[-1]]

  if save is not None and osp.exists(save):
    print("Loading results from file")
    result = np.loadtxt(save)
    recalls = result[:, 0]
    precisions = result[:, 1]
    f_scores = result[:, 2]
    ax.plot(recalls, precisions, color=color, label=label)
    #
    argmax = np.argmax(f_scores)
    print(f"precision, recall, fscore: {precisions[argmax]:.4f} & {recalls[argmax]:.4f} & {f_scores[argmax]:.4f}")    
    return thresholds
  else:
    precisions, recalls, f_scores = [], [], []
    for threshold in thresholds:
      prediction = prediction_func(threshold, cost)
      TP = np.sum(np.logical_and(prediction, ground_truth))
      TN = np.sum(np.logical_and(np.logical_not(prediction), np.logical_not(ground_truth)))
      FP = np.sum(np.logical_and(prediction, np.logical_not(ground_truth)))
      FN = np.sum(np.logical_and(np.logical_not(prediction), ground_truth))
      if ((TP + FP) > 0) and ((TP + FN) > 0):
        print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{TP / (TP + FP):>10.2f}, R:{TP / (TP + FN):>10.2f}")
        precisions.append(TP / (TP + FP))
        recalls.append(TP / (TP + FN))
        f_scores.append(2 * precisions[-1] * recalls[-1] / (precisions[-1] + recalls[-1]))
      else:
        print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{0:>10.2f}, R:{0:>10.2f}")
    ax.plot(recalls, precisions, color=color, label=label)
    if save is not None:
      np.savetxt(save, np.stack((np.array(recalls), np.array(precisions), np.array(f_scores)), axis=1), fmt="%.4f")

  return thresholds


PREDICTION_FUNC = None
GROUND_TRUTH = None
def global_compute_pr(threshold):
  global GROUND_TRUTH, PREDICTION_FUNC
  prediction_func = PREDICTION_FUNC
  ground_truth = GROUND_TRUTH
  prediction = prediction_func(threshold)
  TP = np.sum(np.logical_and(prediction, ground_truth))
  TN = np.sum(np.logical_and(np.logical_not(prediction), np.logical_not(ground_truth)))
  FP = np.sum(np.logical_and(prediction, np.logical_not(ground_truth)))
  FN = np.sum(np.logical_and(np.logical_not(prediction), ground_truth))
  if ((TP + FP) > 0) and ((TP + FN) > 0):
    print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{TP / (TP + FP):>10.2f}, R:{TP / (TP + FN):>10.2f}")
    # precisions.append(TP / (TP + FP))
    # recalls.append(TP / (TP + FN))
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)
    return (precision, recall)
  else:
    print(f"{threshold:>10.2f} TP:{TP:>10} TN:{TN:>10} FP:{FP:>10} FN:{FN:>10}, P:{0:>10.2f}, R:{0:>10.2f}")
    return None, None

def plot_precision_recall_multiproc(ax, cost, ground_truth, prediction_func=None,
                                    thresholds=None, color=None, label=None, save=None):
  if thresholds is None:
    sorted_cost = np.sort(cost)
    thresholds = [sorted_cost[0]-1.0] + [sorted_cost[i] for i in range(0, len(sorted_cost), int(len(sorted_cost) / 50))] + [sorted_cost[-1]]

  if save is not None and osp.exists(save):
    print("Loading results from file")
    result = np.loadtxt(save)
    recalls = result[:, 0]
    precisions = result[:, 1]
    f_scores = result[:, 2]
    ax.plot(recalls, precisions, color=color, label=label)
    #
    argmax = np.argmax(f_scores)
    print(f"precision, recall, fscore: {precisions[argmax]:.4f} & {recalls[argmax]:.4f} & {f_scores[argmax]:.4f}")
    return thresholds
  else:
    precisions, recalls, f_scores = [], [], []
    global PREDICTION_FUNC, GROUND_TRUTH
    PREDICTION_FUNC = prediction_func
    GROUND_TRUTH = ground_truth
    with Pool(4) as p:
      result = p.map(global_compute_pr, thresholds)

    for (p, r) in result:
      if p is not None and r is not None:
        precisions.append(p)
        recalls.append(r)
        f_scores.append(2 * precisions[-1] * recalls[-1] / (precisions[-1] + recalls[-1]))

    ax.plot(recalls, precisions, color=color, label=label)
    if save is not None:
      np.savetxt(save, np.stack((np.array(recalls), np.array(precisions), np.array(f_scores)), axis=1), fmt="%.4f")

  return thresholds

FILTERED_QUERIES = None
FILTERED_NEIGHBORS = None
SUPPORT_THRESHOLD = None
SUPPORT_VARIANCE = None
def global_get_label_base(threshold, costs):
  global FILTERED_QUERIES, FILTERED_NEIGHBORS
  queries = FILTERED_QUERIES
  neighbors = FILTERED_NEIGHBORS

  labels = []
  for i in range(len(queries)):
    labels.append(support_filter(threshold, queries[i], costs[i], neighbors[i], SUPPORT_THRESHOLD, SUPPORT_VARIANCE))
  return np.concatenate(labels, axis=0)

FILTERED_COSTS = None
def global_get_label(threshold):
  return global_get_label_base(threshold, FILTERED_COSTS)

FILTERED_COSTS_PR = None
def global_get_label_pr(threshold):
  return global_get_label_base(threshold, FILTERED_COSTS_PR)

## NOTE: this is plotted from not including any ground truth points
def main(data_dir_base, dest):
  fig, axs = setup_figure(1, 3, 1.0, 1.0, l=0.12, r=0.05, b=0.4, t=0.1, w=0.5, h=0.25)

  rows = ['parkinglot', 'marsdome', 'grove']
  for i, row in enumerate(rows):
    ## load data
    data_dir = osp.join(osp.normpath(data_dir_base), row, "main.fake_obstacle/graph/data")
    print(f"Row {i} corresponds to {row} with data in {data_dir}")
    data_name = "change_detection_v2_result"
    data_file = f"{data_dir}/{data_name}/{data_name}_0.db3"
    data_parser = BagFileParser(data_file)
    data_iter = data_parser.get_bag_msgs_iter(data_name)

    # filter out
    filtered_queries = []
    filtered_results = []
    filtered_centroids = []
    filtered_normals = []
    filtered_rougnhesses = []
    filtered_costs = []
    filtered_rougnhesses_pr = []
    filtered_costs_pr = []
    filtered_neighbors = []

    num_data = 0
    for time, msg in data_iter:
      num_data += 1
      # NOTE: use 450 frames for evaluation only (for now)
      if num_data >= 450:
        break

      #
      query, centroid, normal, result = get_result_from_msg(msg)
      # valid distance and roughness
      filter = np.logical_and.reduce((result[:, 0] >= 0, result[:, 1] > 0))
      # apply filtering
      ftd_query = query[filter]
      ftd_centroid = centroid[filter]
      ftd_normal = normal[filter]
      ftd_result = result[filter]

      filtered_queries.append(ftd_query)
      filtered_results.append(ftd_result)
      filtered_centroids.append(ftd_centroid)
      filtered_normals.append(ftd_normal)

      # do some preprocessing for smoothing and cost computation
      ftd_roughness, ftd_cost = compute_cost(ftd_query, ftd_centroid, ftd_normal, ftd_result)
      ftd_roughness_pr, ftd_cost_pr = compute_cost_prior(ftd_query, ftd_centroid, ftd_normal, ftd_result)
      ftd_neighbors = find_neighbors(ftd_query)
      filtered_rougnhesses.append(ftd_roughness)
      filtered_costs.append(ftd_cost)
      filtered_rougnhesses_pr.append(ftd_roughness_pr)
      filtered_costs_pr.append(ftd_cost_pr)
      filtered_neighbors.append(ftd_neighbors)
    print(f"Loaded {num_data} data points")

    # local versions
    def get_label_base(threshold, costs,
        support_threshold = 2.5,
        support_variance = 0.05,
        queries = filtered_queries,
        neighbors = filtered_neighbors):
      labels = []
      for i in range(len(queries)):
        labels.append(support_filter(threshold, queries[i], costs[i], neighbors[i], support_threshold, support_variance))
      return np.concatenate(labels, axis=0)
    get_label = lambda threshold, *args, costs = filtered_costs, **kwargs: get_label_base(threshold, costs, **kwargs)
    get_label_pr = lambda threshold, *args, costs = filtered_costs_pr, **kwargs: get_label_base(threshold, costs, **kwargs)

    # for global versions
    global FILTERED_QUERIES, FILTERED_NEIGHBORS, FILTERED_COSTS, FILTERED_COSTS_PR, SUPPORT_THRESHOLD, SUPPORT_VARIANCE
    SUPPORT_THRESHOLD = 2.5
    SUPPORT_VARIANCE = 0.05
    FILTERED_QUERIES = filtered_queries
    FILTERED_NEIGHBORS = filtered_neighbors
    FILTERED_COSTS = filtered_costs
    FILTERED_COSTS_PR = filtered_costs_pr


    filtered_results = np.concatenate(filtered_results, axis=0)
    filtered_centroids = np.concatenate(filtered_centroids, axis=0)
    filtered_normals = np.concatenate(filtered_normals, axis=0)
    filtered_rougnhesses = np.concatenate(filtered_rougnhesses, axis=0)
    filtered_costs = np.concatenate(filtered_costs, axis=0)
    filtered_rougnhesses_pr = np.concatenate(filtered_rougnhesses_pr, axis=0)
    filtered_costs_pr = np.concatenate(filtered_costs_pr, axis=0)

    # the current axis
    ax = axs[i]

    #
    os.makedirs(osp.join(dest, 'pr_curves', row), exist_ok=True)

    # precision recall curve
    print("Likelihood")
    result_txt = osp.join(dest, 'pr_curves' , row, 'no_prior_no_filtering.txt')
    thresholds = plot_precision_recall(ax, filtered_costs, filtered_results[:, 3]>0.5, color='r', label="No Prior \& Filtering", save=result_txt)
    # downsampled_thresholds = thresholds[:1]
    # for th in thresholds[1:]:
    #   if np.abs(th - downsampled_thresholds[-1]) > 0.1:
    #     downsampled_thresholds.append(th)
    downsampled_thresholds = thresholds

    print("Likelihood + Filtering")
    result_txt = osp.join(dest, 'pr_curves' , row, 'no_prior.txt')
    # plot_precision_recall(ax, filtered_costs, filtered_results[:, 3]>0.5, get_label, downsampled_thresholds, color='g', label="No Prior", save=result_txt)
    plot_precision_recall_multiproc(ax, filtered_costs, filtered_results[:, 3]>0.5, global_get_label, downsampled_thresholds, color='g', label="No Prior", save=result_txt)

    print("Posterior Predictive")
    result_txt = osp.join(dest, 'pr_curves' , row, 'no_filtering.txt')
    thresholds = plot_precision_recall(ax, filtered_costs_pr, filtered_results[:, 3]>0.5, color='b', label="No Filtering", save=result_txt)
    # downsampled_thresholds = thresholds[:1]
    # for th in thresholds[1:]:
    #   if np.abs(th - downsampled_thresholds[-1]) > 0.1:
    #     downsampled_thresholds.append(th)
    downsampled_thresholds = thresholds

    print("Posterior Predictive + Filtering")
    result_txt = osp.join(dest, 'pr_curves' , row, 'proposed.txt')
    # plot_precision_recall(ax, filtered_costs_pr, filtered_results[:, 3]>0.5, get_label_pr, downsampled_thresholds, color='m', label="Proposed", save=result_txt)
    plot_precision_recall_multiproc(ax, filtered_costs_pr, filtered_results[:, 3]>0.5, global_get_label_pr, downsampled_thresholds, color='m', label="Proposed", save=result_txt)

    axs[i].set_axisbelow(True)
    axs[i].grid(which='both', linestyle='--', alpha=0.75)
    axs[i].set_xticks([0.4, 0.6, 0.8, 1.0])
    axs[i].set_yticks([0.4, 0.6, 0.8, 1.0])

    axs[i].set_xlim(0.4, 1.0)
    axs[i].set_ylim(0.4, 1.0)
    axs[i].set_xlabel("Recall")
    axs[i].set_ylabel("Precision")

  axs[0].set_title('Parking Lot Route')
  axs[1].set_title('MarsDome Route')
  axs[2].set_title('Grove Route')

  handles, labels = axs[0].get_legend_handles_labels()
  fig.legend(handles, labels, loc='lower center', ncol=4, frameon=False)

  plt.savefig(os.path.join(dest, 'terrain_pr_curves.pdf'), pad_inches=0.0, bbox_inches='tight')
  plt.close()
  # plt.show()

if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <path>/graph/data/<database directory>
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')
  parser.add_argument('--dest', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path, args.dest)