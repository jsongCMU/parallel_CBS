import argparse
from pathlib import Path
import os
import re


if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_dir', type=str, default='../instances/evaluation')
    parser.add_argument('--num_agents', type=int, default=20)
    parser.add_argument('--num_instances', type=int, default=1)
    args = parser.parse_args()

    os.system('mkdir -p logs')
    os.system('rm -rf logs/*')


    for file in Path(args.eval_dir).glob("eval_*"):
        
        print("Running ", file.name)
        agents = file.name.split('_')[1][1:]
        instance = file.name.split('_')[2][1:][:-4]
        
        log_file = f'../outputs/eval_n{agents}_i{instance}_result.txt'
        cmd = f'../build/parallelCBS {file} -s > {log_file}'
        ret = os.system(cmd)
        assert ret == 0, 'ERROR -- parallelCBS exited with errors'

        # compare(output_file, f'src/benchmark-files/{scene_name}-ref.txt')
        # t = float(re.findall(
        #     r'total simulation time: (.*?)s', open(log_file).read())[0])
        # print(f'total simulation time: {t:.6f}s')
        # perfs[i][j] = t


    # for i in range(args.num_instances):
    #     print("*** Create random start + goal locations ***")
    #     starts, goals = create_random_start_goal(map, args.num_agents)

    #     save_new_mapf_instance(f'../instances/evaluation/eval_n{args.num_agents}_i{i}.txt', map, starts, goals)