import os
import subprocess

poli_files_dir = 'output/intermediate/'
exe_path = 'NNTest.exe'
root_dir = curr_dir = os.path.dirname(__file__)

args = '-arg_file= args/ball_int_poli_eval.txt'

os.chdir(root_dir)
files = os.listdir(poli_files_dir)

actor_files = []
critic_files = []
for f in files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        if '_critic' in filename:
            critic_files.append(f)
        else:
            actor_files.append(f)

num_files = len(actor_files)

print('model files:')
for f in range(0, num_files):
    print(actor_files[f] + ' ' + critic_files[f])
print('\n')

os.chdir(root_dir)


for f in range(0, num_files):
    command = root_dir + '\\' + exe_path + ' ' + args
    command += ' ' + '-model_file=' + ' ' + poli_files_dir + actor_files[f] \
               + ' ' + '-critic_model_file=' + ' ' + poli_files_dir + critic_files[f]
    print(command + '\n')
    subprocess.call(command)
