import argparse
import numpy as np
import os
import torch
import logging
import sys
import importlib

# ... (import statements and definitions)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
sys.path.append(os.path.join(ROOT_DIR, 'models'))


def parse_args():
    '''PARAMETERS'''
    parser = argparse.ArgumentParser('Testing')
    parser.add_argument('--use_cpu', action='store_true', default=False, help='use cpu mode')
    parser.add_argument('--gpu', type=str, default='0', help='specify gpu device')
    parser.add_argument('--batch_size', type=int, default=24, help='batch size in training')
    parser.add_argument('--num_category', default=40, type=int, choices=[10, 40],  help='training on ModelNet10/40')
    parser.add_argument('--num_point', type=int, default=1024, help='Point Number')
    parser.add_argument('--log_dir', type=str, required=True, help='Experiment root')
    parser.add_argument('--use_normals', action='store_true', default=False, help='use normals')
    parser.add_argument('--use_uniform_sample', action='store_true', default=False, help='use uniform sampiling')
    parser.add_argument('--num_votes', type=int, default=3, help='Aggregate classification scores with voting')
    return parser.parse_args()


def test(model, loader, num_class=40, vote_num=1):
    mean_correct = []
    classifier = model.eval()
    class_acc = np.zeros((num_class, 3))

    for j, (points, target) in tqdm(enumerate(loader), total=len(loader)):
        if not args.use_cpu:
            points, target = points.cuda(), target.cuda()

        points = points.transpose(2, 1)
        vote_pool = torch.zeros(target.size()[0], num_class).cuda()

        for _ in range(vote_num):
            pred, _ = classifier(points)
            vote_pool += pred
        pred = vote_pool / vote_num
        pred_choice = pred.data.max(1)[1]

        for cat in np.unique(target.cpu()):
            classacc = pred_choice[target == cat].eq(target[target == cat].long().data).cpu().sum()
            class_acc[cat, 0] += classacc.item() / float(points[target == cat].size()[0])
            class_acc[cat, 1] += 1
        correct = pred_choice.eq(target.long().data).cpu().sum()
        mean_correct.append(correct.item() / float(points.size()[0]))

    class_acc[:, 2] = class_acc[:, 0] / class_acc[:, 1]
    class_acc = np.mean(class_acc[:, 2])
    instance_acc = np.mean(mean_correct)
    return instance_acc, class_acc


def main(args):
    def log_string(str):
        logger.info(str)
        print(str)

    '''HYPER PARAMETER'''
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu

    '''CREATE DIR'''
    experiment_dir = 'log/classification/' + args.log_dir

    '''LOG'''
    logger = logging.getLogger("Model")
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler = logging.FileHandler('%s/eval.txt' % experiment_dir)
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    log_string('PARAMETER ...')
    log_string(args)

    '''MODEL LOADING'''
    num_class = args.num_category
    model_name = os.listdir(experiment_dir + '/logs')[0].split('.')[0]
    model = importlib.import_module(model_name)

    classifier = model.get_model(num_class, normal_channel=args.use_normals)
    if not args.use_cpu:
        classifier = classifier.cuda()

    checkpoint = torch.load(str(experiment_dir) + '/checkpoints/best_model.pth')
    classifier.load_state_dict(checkpoint['model_state_dict'])

    '''LOAD YOUR POINT CLOUD FILE'''
    # Load your .txt point cloud file here and preprocess it
    # Assuming your point cloud is loaded into 'points' as a numpy array
    # points = ...  # Load and preprocess your point cloud data

    # # Convert numpy array to a PyTorch tensor
    # points_tensor = torch.from_numpy(points).unsqueeze(0)  # Adding a batch dimension
    
    txt_file_path = 'person_0001.txt'

    # Load the data from the text file
    with open(txt_file_path, 'r') as file:
        lines = file.readlines()

    # Convert the lines into a numpy array (assuming comma-separated values)
    points = np.array([[float(val) for val in line.strip().split(',')] for line in lines])

    # Normalize the points like in ModelNetDataLoader
    points[:, :3] = pc_normalize(points[:, :3])
    
    points_tensor = torch.from_numpy(points).unsqueeze(0)  # Adding a batch dimension



    '''INFERENCE ON THE SINGLE POINT CLOUD'''
    with torch.no_grad():
        if not args.use_cpu:
            points_tensor = points_tensor.cuda()
        points_tensor = points_tensor.transpose(2, 1)
        pred, _ = classifier(points_tensor)
        pred_choice = pred.data.max(1)[1]

    log_string('Predicted Class: %d' % pred_choice.item())
    
def pc_normalize(pc):
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc**2, axis=1)))
    pc = pc / m
    return pc

if __name__ == '__main__':
    args = parse_args()
    main(args)
