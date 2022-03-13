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
  roughness = np.empty((num_points, 1))
  roughness[:, 0] = data['flex14']
  #
  return query, centroid, normal, roughness


def main(data_dir):
  #
  data_dir = osp.join(osp.normpath(data_dir), "graph/data")
  data_name = "change_detection_result"
  data_file = f"{data_dir}/{data_name}/{data_name}_0.db3"
  data_parser = BagFileParser(data_file)
  data_iter = data_parser.get_bag_msgs_iter("change_detection_result")

  fig, ax = plt.subplots(1, 1, figsize=(6, 6))

  #
  filtered_roughnesses = []
  for time, msg in data_iter:
    #
    query, centroid, normal, roughness = get_result_from_msg(msg)
    #
    filter = np.logical_and.reduce((roughness[:, 0] > 0,  query[:, 2] < 0.5))
    # filter = (roughness[:, 0] > 0)
    filtered_roughness = roughness[filter].flatten()
    filtered_roughnesses.append(filtered_roughness)

  filtered_roughnesses = np.concatenate(filtered_roughnesses)
  print(len(filtered_roughnesses))
  N = len(filtered_roughnesses)
  weights = np.ones(N) / float(N)
  y, _, _ = ax.hist(filtered_roughnesses.squeeze(), bins=20, range=(0, 1), color='b', weights=weights)

  plt.show()

if __name__ == "__main__":

  parser = argparse.ArgumentParser()

  # Assuming following path structure:
  # <path>/graph/data/<database directory>
  parser.add_argument('--path', default=os.getcwd(), type=str, help='path to vtr folder (default: os.getcwd())')

  args = parser.parse_args()

  main(args.path)