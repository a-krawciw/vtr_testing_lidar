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
    'font.size': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'axes.linewidth': 1,
    'axes.titlesize': 10,
    'axes.labelsize': 10,
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

## NOTE: this is plotted from not including any ground truth points
def main(data_dir_base, dest):
  fig, axs = setup_figure(2, 3, 1.0, 1.0, l=0.10, r=0.05, b=0.07, t=0.07, w=0.33, h=0.30)

  rows = ['parkinglot', 'marsdome', 'grove']
  for i, row in enumerate(rows):
    ## load data
    data_dir = osp.join(osp.normpath(data_dir_base), row, "main.no_fake_obstacle/graph/data")
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
    filtered_neighbors = []
    for time, msg in data_iter:
      #
      query, centroid, normal, result = get_result_from_msg(msg)
      # valid distance and roughness
      filter = np.logical_and.reduce((result[:, 0] >= 0, result[:, 1] > 0, centroid[:, 2] < 0.0))
      # apply filtering
      ftd_query = query[filter]
      ftd_centroid = centroid[filter]
      ftd_normal = normal[filter]
      ftd_result = result[filter]

      filtered_queries.append(ftd_query)
      filtered_results.append(ftd_result)
      filtered_centroids.append(ftd_centroid)
      filtered_normals.append(ftd_normal)

    filtered_results = np.concatenate(filtered_results, axis=0)
    filtered_centroids = np.concatenate(filtered_centroids, axis=0)
    filtered_normals = np.concatenate(filtered_normals, axis=0)

    normalz = np.arccos(np.clip(np.abs(filtered_normals[:, 2]), 0., 1.)) / np.pi * 180.
    roughness = filtered_results[:, 1]
    N = len(normalz)
    weights = np.ones(N) / float(N)

    y0, _, _ = axs[0, i].hist(normalz, bins=20, range=(0, 45.0), color='b', weights=weights, rwidth=0.7)
    y1, _, _ = axs[1, i].hist(roughness, bins=20, range=(0, 0.2), color='b', weights=weights, rwidth=0.7)
    axs[0, i].set_axisbelow(True)
    axs[1, i].set_axisbelow(True)
    axs[0, i].grid(which='both', linestyle='--', alpha=0.75)
    axs[1, i].grid(which='both', linestyle='--', alpha=0.75)
    axs[0, i].set_xticks([0., 7.5, 15., 22.5, 30., 37.5, 45.])
    axs[1, i].set_xticks([0., 0.05, 0.1, 0.15, 0.2])

    labels = axs[0, i].xaxis.get_ticklabels()
    for j in range(len(labels)):
      label = labels[j]
      if j > 0 and j < len(labels) - 1 and (j % np.floor(len(labels) / 3) != 0):
        label.set_visible(False)

    labels = axs[1, i].xaxis.get_ticklabels()
    for j in range(len(labels)):
      label = labels[j]
      if j > 0 and j < len(labels) - 1 and (j % np.floor(len(labels) / 2) != 0):
        label.set_visible(False)

    axs[0, i].set_ylim(0, 0.45)
    axs[1, i].set_ylim(0, 0.55)

  axs[0, 0].set_ylabel(r'Slope (deg)')
  axs[1, 0].set_ylabel(r'Roughness ($\mathrm{m^2}$)')
  axs[0, 0].set_title('Parking Lot Route')
  axs[0, 1].set_title('MarsDome Route')
  axs[0, 2].set_title('Grove Route')

  plt.savefig(os.path.join(dest, 'terrain_rough_slope_hist.pdf'), pad_inches=0.0, bbox_inches='tight')
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