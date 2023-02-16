import torch
# import tensorflow as tf
import numpy as np
import os
import matplotlib.pyplot as plt
from torch import nn

'''
    *
    *

PREPARING DATASET

    *
    *
'''


class AllDataset(torch.utils.data.Dataset):
    '''
    Prepare the dataset for regression
    '''

    def __init__(self, X, y, scale_data=True):
        # data cant be tensor format
        if not torch.is_tensor(X) and not torch.is_tensor(y):
            # Apply scaling if necessary
            # if scale_data:
            # X = StandardScaler().fit_transform(X)  # standardization data
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
            nn.ReLU(),
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

TRAINNING

    *
    *
'''

if __name__ == '__main__':

    # Set fixed random number seed
    torch.manual_seed(42)

    # define all data path
    path = "C:/Users/gabri/OneDrive/Área de Trabalho/IC_Jonathan/Dataset"
    dir = os.listdir(path)

    # Variable inicialization
    [textSplit, value] = [[], []]  # auxiliar variable
    old_text_size, k = 0, 0  # auxiliar variable

    [X_left, y_left] = [[], []]  # data variable
    [X_right, y_right] = [[], []]  # data variable

    [X_max_left, y_max_left, X_min_left, y_min_left] = [
        [0, 0, 0], 0, [0, 0, 0], 0]  # normalization variable
    [X_max_right, y_max_right, X_min_right, y_min_right] = [
        [0, 0, 0], 0, [0, 0, 0], 0]  # normalization variable

    for i in dir:
        arch = []
        if (not i.startswith('.')):  # verify if is a valid directory
            # list all the archives in directory
            arch = sorted(os.listdir(path + '/' + i))
            for j in arch:
                if (not j.startswith('.')):  # verify if is a valid archive
                    if ('left' in j):  # verifcation by archive name
                        k = 1
                    elif ('right' in j):  # verifcation by archive name
                        k = 4
                    elif ('left' not in j and 'right' not in j):  # verification by archive name
                        continue
                    # store the final data path
                    final_path = (path+'/'+i+'/'+j)

                    # Load dataset
                    with open(final_path, 'r') as archive:
                        text = archive.read()

                    # count how many data is in archive
                    text_size = len(text.split('\n'))
                    textSplit.extend(text.split('\n'))  # divide data by line

                    # Separate data
                    for n in range(text_size):
                        # divides the values of each sensor in a vector
                        valSensor = textSplit[n+old_text_size].split('\t')
                        # verify if data have information of sensor
                        if (valSensor != ['']):

                            # left knee data
                            if (k == 1):
                                # devides each sensor value on a position of vector
                                for w in range(3):
                                    if (w == 0):  # position 0 -> sensor 1
                                        if (X_max_left[0] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_left[0] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_left[0] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_left[0] = (
                                                float(valSensor[w+k])*(-1))
                                    if (w == 1):  # position 1 -> sensor 2
                                        if (X_max_left[1] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_left[1] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_left[1] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_left[1] = (
                                                float(valSensor[w+k])*(-1))
                                    if (w == 2):  # position 2 -> sensor 3
                                        if (X_max_left[2] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_left[2] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_left[2] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_left[2] = (
                                                float(valSensor[w+k])*(-1))
                                    value.append(
                                        [(float(valSensor[w+k])*(-1))])  # store the values of 3 sensor in a list
                                X_left.append(value)  # Input
                                if (y_max_left < (float(valSensor[7])*(-1))):
                                    # values to normalization desired output
                                    y_max_left = (float(valSensor[7])*(-1))
                                if (y_min_left > (float(valSensor[7])*(-1))):
                                    # values to normalization desired output
                                    y_min_left = (float(valSensor[7])*(-1))
                                # desired output
                                y_left.append((float(valSensor[7])*(-1)))
                                value = []

                            # right knee data
                            if (k == 4):
                                # devides each sensor value on a position of vector
                                for w in range(3):
                                    if (w == 0):  # position 0 -> sensor 1
                                        if (X_max_right[0] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_right[0] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_right[0] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_right[0] = (
                                                float(valSensor[w+k])*(-1))
                                    if (w == 1):  # position 1 -> sensor 2
                                        if (X_max_right[1] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_right[1] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_right[1] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_right[1] = (
                                                float(valSensor[w+k])*(-1))
                                    if (w == 2):  # position 2 -> sensor 3
                                        if (X_max_right[2] < (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_max_right[2] = (
                                                float(valSensor[w+k])*(-1))
                                        if (X_min_right[2] > (float(valSensor[w+k])*(-1))):
                                            # values to normalization input
                                            X_min_right[2] = (
                                                float(valSensor[w+k])*(-1))
                                    value.append(
                                        [(float(valSensor[w+k])*(-1))])  # store the values of 3 sensor in a list
                                X_right.append(value)  # Input
                                if (y_max_right < (float(valSensor[7])*(-1))):
                                    # values to normalization desired output
                                    y_max_right = (float(valSensor[7])*(-1))
                                if (y_min_right > (float(valSensor[7])*(-1))):
                                    # values to normalization desired output
                                    y_min_right = (float(valSensor[7])*(-1))
                                # desired output
                                y_right.append((float(valSensor[7])*(-1)))
                                value = []

                    # old_text_size = text_size
                    old_text_size += text_size

    # Creating arrays to recieved input and desired output
    X_all_left = np.ndarray(shape=(len(X_left), 3), dtype=float)
    y_all_left = np.ndarray(shape=(len(y_left), 1), dtype=float)

    X_all_right = np.ndarray(shape=(len(X_right), 3), dtype=float)
    y_all_right = np.ndarray(shape=(len(y_right), 1), dtype=float)

    # normalizing left knee data
    for i in range(len(X_left)):
        for j in range(3):
            X_all_left[i][j] = (
                (float(X_left[i][j][0]) - X_min_left[j]) / (X_max_left[j] - X_min_left[j]))  # input
        y_all_left[i] = ((float(y_left[i]) - y_min_left) /
                         (y_max_left - y_min_left))  # desired output

    # normalizing right knee data
    for i in range(len(X_right)):
        for j in range(3):
            X_all_right[i][j] = ((float(
                X_right[i][j][0]) - X_min_right[j]) / (X_max_right[j] - X_min_right[j]))  # input
        y_all_right[i] = ((float(y_right[i]) - y_min_right) /
                          (y_max_right - y_min_right))  # desired output

    # configure train data
    # send 70% of all data to train
    dataset_left = AllDataset(
        X_all_left[0:int(len(X_all_left)*0.7)], y_all_left[0:int(len(y_all_left)*0.7)])
    dataset_right = AllDataset(X_all_right[0:int(
        len(X_all_right)*0.7)], y_all_right[0:int(len(y_all_right)*0.7)])

    # configure trainloader (hw the data will be trained)
    trainloader_left = torch.utils.data.DataLoader(
        dataset_left, batch_size=750, shuffle=True, num_workers=1)
    trainloader_right = torch.utils.data.DataLoader(
        dataset_right, batch_size=750, shuffle=True, num_workers=1)

    # Initialize the MLP
    mlp_left = MLP()  # NN to left knee data
    mlp_right = MLP()  # NN to right knee data

    # Define the loss function and optimizer (Adam)
    # loss_function = nn.MSELoss()
    # loss_function = nn.L1Loss()
    optimizer_left = torch.optim.Adam(
        mlp_left.parameters(), lr=1e-3)  # learning rate (alpha) = 1e-3
    optimizer_right = torch.optim.Adam(
        mlp_right.parameters(), lr=1e-3)  # learning rate (alpha) = 1e-3

    [loss_array_left, loss_array_right] = [[], []]

    # Run the training loop to left knee data
    for epoch in range(0, 70):  # number of epochs

        # Print epoch
        print(f'Starting epoch {epoch+1}')

        # Set current loss value
        current_loss = 0.0
        # Iterate over the DataLoader for training data
        for i, data in enumerate(trainloader_left, 0):

            # Get and prepare inputs
            inputs, targets = data
            inputs, targets = inputs.float(), targets.float()
            targets = targets.reshape((targets.shape[0], 1))

            # Perform forward pass
            outputs = mlp_left(inputs)

            # Compute loss
            # loss = loss_function(outputs, targets)
            loss = (0.5*(targets-outputs)**2).mean()

            # Zero the gradients
            optimizer_left.zero_grad()

            # Perform backward pass
            loss.backward()

            # Perform optimization
            optimizer_left.step()

            # Print statistics
            current_loss += loss.item()
            # if i % 10 == 0:
            #   print('Loss after mini-batch %5d: %.3f' %
            #        (i + 1, current_loss/1))
            # current_loss = 0.0
        loss_array_left.append(current_loss)
    # Process is complete.
    print('Training left knee data process has finished.\n\n')

    # Run the training loop to right knee data
    for epoch in range(0, 70):  # number of epochs

        # Print epoch
        print(f'Starting epoch {epoch+1}')

        # Set current loss value
        current_loss = 0.0
        # Iterate over the DataLoader for training data
        for i, data in enumerate(trainloader_right, 0):

            # Get and prepare inputs
            inputs, targets = data
            inputs, targets = inputs.float(), targets.float()
            targets = targets.reshape((targets.shape[0], 1))

            # Perform forward pass
            outputs = mlp_right(inputs)

            # Compute loss
            # loss = loss_function(outputs, targets)
            loss = (0.5*(targets-outputs)**2).mean()

            # Zero the gradients
            optimizer_right.zero_grad()

            # Perform backward pass
            loss.backward()

            # Perform optimization
            optimizer_right.step()

            # Print statistics
            current_loss += loss.item()
            # if i % 10 == 0:
            #   print('Loss after mini-batch %5d: %.3f' %
            #        (i + 1, current_loss/1))
            # current_loss = 0.0
        loss_array_right.append(current_loss)
    # Process is complete.
    print('Training right knee data process has finished.')

    # plot left knee data loss
    plt.figure(1)
    plt.plot(loss_array_left)
    plt.title('left knee data loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.show()

    # plot right knee data loss
    plt.figure(2)
    plt.plot(loss_array_right)
    plt.title('right knee data loss')
    plt.ylabel('loss')
    plt.xlabel('epoch')
    plt.show()

    '''
        *
        *

    TEST

        *
        *
    '''

    # Separate 30% of data to test
    X_test_left = X_all_left[(int(len(X_all_left)*0.7)+1):]
    y_test_left = y_all_left[(int(len(y_all_left)*0.7)+1):]

    X_test_right = X_all_right[(int(len(X_all_right)*0.7)+1):]
    y_test_right = y_all_right[(int(len(y_all_right)*0.7)+1):]

    # transform ndarray to tensor
    X_test_left_tensor = torch.as_tensor(X_test_left, dtype=torch.float)
    y_test_left_tensor = torch.as_tensor(y_test_left, dtype=torch.float)

    X_test_right_tensor = torch.as_tensor(X_test_right, dtype=torch.float)
    y_test_right_tensor = torch.as_tensor(y_test_right, dtype=torch.float)

    # testing data on NN
    output_left = mlp_left(X_test_left_tensor)
    output_right = mlp_right(X_test_right_tensor)

    # plotting left knee angle data
    fig, ax = plt.subplots(3)
    fig.set_figwidth(10)
    fig.set_figheight(10)

    ax[0].scatter(output_left[:].detach().numpy(),
                  y_test_left_tensor[:].detach().numpy())
    ax[1].plot(output_left[:].detach().numpy())
    ax[2].plot(y_test_left_tensor[:].detach().numpy())

    ax[0].set_title('Correlation NN-output x desired-outputs (left knee)')
    ax[1].set_title('NN-output')
    ax[2].set_title('Desired-outputs')
    '''
     ax[1].set_xlim(1000, 3000)
     ax[2].set_xlim(1000, 3000)
    '''
    plt.show()

    # plotting right knee angle data
    fig2, ax2 = plt.subplots(3)
    fig2.set_figwidth(10)
    fig2.set_figheight(10)

    ax2[0].scatter(output_right[:].detach().numpy(),
                   y_test_right_tensor[:].detach().numpy())
    ax2[1].plot(output_right[:].detach().numpy())
    ax2[2].plot(y_test_right_tensor[:].detach().numpy())
    '''
     ax2[1].set_xlim(1000, 3000)
     ax2[2].set_xlim(1000, 3000)
    '''
    ax2[0].set_title('Correlation NN-output x desired-outputs (right knee)')
    ax2[1].set_title('NN-output')
    ax2[2].set_title('Desired-outputs')
    plt.show()
