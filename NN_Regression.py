'''
    Regression NN to measure knees angles
'''

import os
import torch
import random
import numpy as np
import pandas as pd
from torch import nn
import tensorflow as tf
import matplotlib.pyplot as plt
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import r2_score

'''
*
*
  Prepare the dataset for regression
*
*
'''
class AllDataset(torch.utils.data.Dataset):
  def __init__(self, X, y, scale_data=True):
    if not torch.is_tensor(X) and not torch.is_tensor(y): #data cant be tensor format
      # Apply scaling if necessary
      #if scale_data:
         # X = StandardScaler().fit_transform(X) #standardization data
      self.X = torch.from_numpy(X)
      self.y = torch.from_numpy(y)

  def __len__(self):
      return len(self.X)

  def __getitem__(self, i):
      return self.X[i], self.y[i]


'''
*
*
    Creating Multilayer Perceptron
*
*
'''
class MLP(nn.Module):
  '''
    Multilayer Perceptron for regression.
  '''
  def __init__(self):
    super().__init__()
    self.layers = nn.Sequential(
      nn.Linear(3, 10),
      #nn.Dropout(p=0.2),
      nn.Tanh(),
      #nn.Dropout(p=0.2),
      nn.Linear(10, 1)
    )

  def forward(self, x):
    '''
      Forward pass
    '''
    return self.layers(x)

'''
*
*
    Configure data
*
*
'''
path = 'DATA_PATH'
dir = os.listdir(path) #dataset

X_all_left = []
X_all_right = []

[sensor_all_left, sensor_all_right, xsens_all_left, xsens_all_right] = [[], [], [], []]
[X_all_right, X_all_left, y_all_right, y_all_left] = [[], [], [], []]

for i in dir:
  arch = []
  if(not i.startswith('.')):
    arch = sorted(os.listdir(path+ '/' + i)) #tipos de teste e user
    for j in arch: #tipos de teste
      txt_archives = path+'/'+i+'/'+j
      sensors_values = pd.read_csv(txt_archives, sep='\s+', header=0)

      sensor_all_left.append(sensors_values.iloc[:,1:4])
      sensor_all_right.append(sensors_values.iloc[:,4:7])
      xsens_all_left.append(sensors_values.iloc[:, 7])
      xsens_all_right.append(sensors_values.iloc[:, 8])

num_files = len(arch)
for i in range(num_files):
  for j in range(len(sensor_all_right[i])):
    X_all_right.append(sensor_all_right[i].iloc[j, :])
    X_all_left.append(sensor_all_left[i].iloc[j, :])
    y_all_right.append(xsens_all_right[i].iloc[j])
    y_all_left.append(xsens_all_left[i].iloc[j])

[X_max_left, X_max_right, X_min_left, X_min_right] = [[0, 0, 0], [0, 0, 0], [1000, 1000, 1000], [1000, 1000, 1000]]
[y_max_left, y_max_right, y_min_left, y_min_right] = [0, 0, 1000, 1000]

for i in range(len(X_all_left)):
  for j in range(3):
    if(X_max_left[j] < X_all_left[i][j]): X_max_left[j] = X_all_left[i][j]
    if(X_min_left[j] > X_all_left[i][j]): X_min_left[j] = X_all_left[i][j]
    if(X_max_right[j] < X_all_right[i][j]): X_max_right[j] = X_all_right[i][j]
    if(X_min_right[j] > X_all_right[i][j]): X_min_right[j] = X_all_right[i][j]
  if(y_max_left < y_all_left[i]): y_max_left = y_all_left[i]
  if(y_min_left > y_all_left[i]): y_min_left = y_all_left[i]
  if(y_max_right < y_all_right[i]): y_max_right = y_all_right[i]
  if(y_min_right > y_all_right[i]): y_min_right = y_all_right[i]

# normalizing right knee data
for i in range(len(X_all_right)):
  for j in range(3):
    #X_all_right[i][j] = (X_all_right[i][j] - X_min_right[j]) / ((X_max_right[j] - X_min_right[j])) #input
    X_all_right[i][j] = ((2*(X_all_right[i][j] - X_min_right[j])) / (X_max_right[j] - X_min_right[j])) - 1 #input
    #X_all_right[i][1] = 0;
  #y_all_right[i] = ((y_all_right[i] - y_min_right) / (y_max_right - y_min_right)) #desired output
  y_all_right[i] = ((2*(y_all_right[i] - y_min_right)) / (y_max_right - y_min_right)) - 1 #desired output

# normalizing left knee data
for i in range(len(X_all_left)):
  for j in range(3):
    X_all_left[i][j] = ((2*(X_all_left[i][j] - X_min_left[j])) / (X_max_left[j] - X_min_left[j])) - 1 #input
    #X_all_left[i][1] = 0;
  y_all_left[i] = ((2*(y_all_left[i] - y_min_left)) / (y_max_left - y_min_left)) - 1 #desired output

#shuffle data
temp_left = list(zip(X_all_left, y_all_left))
temp_right = list(zip(X_all_right, y_all_right))
random.shuffle(temp_left)
random.shuffle(temp_right)
X_shuffle_left, y_shuffle_left = zip(*temp_left)
X_shuffle_right, y_shuffle_right = zip(*temp_right)

#convert to np array data
X_all_left_np = np.array(list(X_shuffle_left), dtype=float)
y_all_left_np = np.array(list(y_shuffle_left), dtype=float)

X_all_right_np = np.array(list(X_shuffle_right), dtype=float)
y_all_right_np = np.array(list(y_shuffle_right), dtype=float)

'''
*
*
    Trainning
*
*
'''
def RSquare(targets, outputs): #function R-square
    target_mean = torch.mean(targets)
    ss_tot = torch.sum((targets - target_mean) ** 2)
    ss_res = torch.sum((targets - outputs) ** 2)
    r2 = 1 - (ss_res / ss_tot)
    return r2

#train left and right

if __name__ == '__main__':

  epoch_left = 3000
  epoch_right = 400

  # Set fixed random number seed
  torch.manual_seed(42)

  #configure train data
  dataset_left = AllDataset(X_all_left_np[0:int(len(X_all_left_np)*0.75)], y_all_left_np[0:int(len(y_all_left_np)*0.75)]) # send only 75% of all data to train
  dataset_right = AllDataset(X_all_right_np[0:int(len(X_all_right_np)*0.75)], y_all_right_np[0:int(len(y_all_right_np)*0.75)]) # send only 75% of all data to train

  trainloader_left = torch.utils.data.DataLoader(dataset_right, batch_size=800, shuffle=True, num_workers=1)
  trainloader_right = torch.utils.data.DataLoader(dataset_right, batch_size=800, shuffle=True, num_workers=1)

  # Initialize the MLP
  mlp_left = MLP() # NN to left knee data

  # Define the loss function and optimizer (Adam)
  #loss_function = nn.MSELoss()
  #loss_function = nn.L1Loss()
  optimizer_left = torch.optim.Adam(mlp_left.parameters(), lr=1e-4) #learning rate (alpha) = 1e-5

  [loss_array_left, loss_array_right] = [[], []]
  [r2_right_tensor, r2_left_tensor] = [[], []]

  # Run the training loop to left knee data
  for epoch in range(0, epoch_left): # number of epochs

    # Print epoch
    print(f'Starting epoch {epoch+1}')

    # Set current loss value
    current_loss_left = 0.0
    # Iterate over the DataLoader for training data
    for i, data in enumerate(trainloader_left, 0):

      # Get and prepare inputs
      inputs, targets = data
      inputs, targets = inputs.float(), targets.float()
      targets = targets.reshape((targets.shape[0], 1))

      # Perform forward pass
      outputs = mlp_left(inputs)

      # R-Square
      r2_left = RSquare(targets, outputs)

      # Compute loss
      loss_left = (0.5*(targets-outputs)**2).mean()

      # Zero the gradients
      optimizer_left.zero_grad()

      # Perform backward pass
      loss_left.backward()

      # Perform optimization
      optimizer_left.step()

      # Print statistics
      current_loss_left += loss_left.item()
      #if i % 10 == 0:
       #   print('Loss after mini-batch %5d: %.3f' %
        #        (i + 1, current_loss/1))
         # current_loss = 0.0
    loss_array_left.append(current_loss_left)

    # store R-square values
    r2_left_tensor.append(r2_left)

  # Process is complete.
  print('Training left knee data process has finished.\n\n')


  mlp_right = MLP() # NN to right knee data
  optimizer_right = torch.optim.Adam(mlp_right.parameters(), lr=1e-4) #learning rate (alpha) = 1e-5

  # Run the training loop to right knee data
  for epoch in range(0, epoch_right): # number of epochs

    # Print epoch
    print(f'Starting epoch {epoch+1}')

    # Set current loss value
    current_loss_right = 0.0
    # Iterate over the DataLoader for training data
    for i, data in enumerate(trainloader_right, 0):

      # Get and prepare inputs
      inputs, targets = data
      inputs, targets = inputs.float(), targets.float()
      targets = targets.reshape((targets.shape[0], 1))

      # Perform forward pass
      outputs = mlp_right(inputs)

      # R-Square
      r2_right = RSquare(targets, outputs)

      # Compute loss
      #loss = loss_function(outputs, targets)
      loss_right = (0.5*(targets-outputs)**2).mean()

      # Zero the gradients
      optimizer_right.zero_grad()

      # Perform backward pass
      loss_right.backward()

      # Perform optimization
      optimizer_right.step()

      # Print statistics
      current_loss_right += loss_right.item()
      #if i % 10 == 0:
       #   print('Loss after mini-batch %5d: %.3f' %
        #        (i + 1, current_loss/1))
         # current_loss = 0.0
    loss_array_right.append(current_loss_right)

    # store R-square values
    r2_right_tensor.append(r2_right)

  # Process is complete.
  print('Training right knee data process has finished.')
