import os
import subprocess

poli_files_dir = 'C:/Users/Jason/Desktop/progs/NNTest/output/intermediate/test/'
root_dir = 'C:/Users/Jason/Desktop/progs/NNTest/'
exe_name = exe_path = 'NNTest.exe'

args = '-arg_file= args/ball_int_poli_eval.txt'

os.chdir(poli_files_dir)
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
    command = root_dir + exe_name + ' ' + args
    command += ' ' + '-model_file=' + ' ' + poli_files_dir + f
    print(command + '\n')
    subprocess.call(command)
