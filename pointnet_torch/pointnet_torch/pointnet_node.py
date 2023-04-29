import numpy as np

import torch
import torch.nn.parallel
import torch.utils.data
from torch.autograd import Variable

from .pointnet import PointNetCls

if torch.cuda.is_available():
    import torch.backends.cudnn as cudnn

# RCLPY packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# Problem ontology
classes_dict = {'Airplane': 0, 'Bag': 1, 'Cap': 2, 'Car': 3, 'Chair': 4, 
                'Earphone': 5, 'Guitar': 6, 'Knife': 7, 'Lamp': 8, 'Laptop': 9,
                'Motorbike': 10, 'Mug': 11, 'Pistol': 12, 'Rocket': 13, 
                'Skateboard': 14, 'Table': 15}

class PointNetNode(Node):

    def __init__(self):
        super().__init__('pointnet_cls_node')

        self.declare_parameter('model_path', '/home/kimsooyoung/Downloads/pointnet/pointnet.pytorch/utils/cls/cls_model_23.pth')
        self.declare_parameter('num_points', 10000)

        MODEL_PATH = self.get_parameter('model_path').value
        NUM_POINTS = self.get_parameter('num_points').value

        # Create the classification network from pre-trained model
        self.classifier = PointNetCls(k=len(classes_dict.items()), num_points=NUM_POINTS)
        if torch.cuda.is_available():
            self.classifier.cuda()
            self.classifier.load_state_dict(torch.load(MODEL_PATH))
        else:
            self.classifier.load_state_dict(torch.load(MODEL_PATH, map_location='cpu'))
        self.classifier.eval()

        self.get_logger().info('PointNet classification network has been loaded')

        self.pc_subscription = self.create_subscription(
            PointCloud2, 'pointcloud',
            self.obj_callback, 10
        )

        self.get_logger().info('PointNet classification Node has been started')

        self.cb_flag = True

    def obj_callback(self, msg):
        # 1. Convert PointCloud2 message to numpy array
        points_list = []
        for data in pc2.read_points(msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])
        points = np.asarray(points_list, dtype=np.float32)

        # 2. numpy resample
        m, n = points.shape
        choice = np.random.choice(m, 10000, replace=True)
        point_set = points[choice, :]

        # 3. numpy to torch
        point_set = torch.from_numpy(point_set)

        # 4. perform inference in GPU
        if self.cb_flag == True:
            points = Variable(point_set.unsqueeze(0))
            points = points.transpose(2, 1)
            if torch.cuda.is_available():
                points = points.cuda()
            pred_logsoft, _ = self.classifier(points)

            # move data back to cpu 
            pred_logsoft_cpu = pred_logsoft.data.cpu().numpy().squeeze()
            pred_soft_cpu = np.exp(pred_logsoft_cpu)
            pred_class = np.argmax(pred_soft_cpu)

            self.get_logger().info(f'Your object is a [{list(classes_dict.keys())[pred_class]}]')
            self.get_logger().info('Probability is a [{:0.3}]'.format(pred_soft_cpu[pred_class]))
        
            self.cb_flag = False

def main(args=None):
    rclpy.init(args=args)

    node = PointNetNode()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
