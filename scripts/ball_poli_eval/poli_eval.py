import os
import subprocess

poli_files_dir = 'output/intermediate/'
exe_path = 'NNTest.exe'

root_dir = curr_dir = os.path.dirname(__file__)
print(root_dir)

args = '-arg_file= args/ball_int_poli_eval.txt'

os.chdir(root_dir)
files = os.listdir(poli_files_dir)

model_files = []
for f in files:
    filename, ext = os.path.splitext(f)
    if (ext == '.h5'):
        model_files.append(f)

print('model files:')
for f in model_files:
    print(f)
print('\n')

os.chdir(root_dir)

for f in model_files:
    command = exe_path + ' ' + args
    command += ' ' + '-model_file=' + ' ' + poli_files_dir + f
    print(command + '\n')
    subprocess.call(command)
