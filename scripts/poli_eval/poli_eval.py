import os
import subprocess

poli_files_dir = 'C:/Users/Jason/Documents/compsci_stuff/c++_stuff/NNTest/data/intermediate/'
root_dir = 'C:/Users/Jason/Documents/compsci_stuff/c++_stuff/NNTest/'
exe_name = exe_path = 'NNTest.exe'

args = '-arg_file= args/int_poli_eval.txt'

os.chdir(poli_files_dir)
files = os.listdir()

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
