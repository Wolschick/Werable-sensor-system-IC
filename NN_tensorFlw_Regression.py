'''
*
Configure data
*
'''

path = '/content/drive/MyDrive/IC_Jonathan/Dataset'
dir = os.listdir(path)

# Variable inicialization
[textSplit, value] = [[], []] # auxiliar variable
old_text_size, k = 0, 0 # auxiliar variable

[X_left, y_left] = [[], []] # data variable
[X_right, y_right] = [[], []] # data variable

[X_max_left, y_max_left, X_min_left, y_min_left] = [[0, 0, 0], 0, [0, 0, 0], 0] # normalization variable
[X_max_right, y_max_right, X_min_right, y_min_right] = [[0, 0, 0], 0, [0, 0, 0], 0] # normalization variable

for i in dir: 
  arch = []
  if(not i.startswith('.')):
    arch = sorted(os.listdir(path+ '/' + i))
    for j in arch:
      if(not j.startswith('.')): 
        if('left' in j): k = 1
        elif('right' in j): k = 4
        elif('left' not in j and 'right' not in j): continue
        final_path = (path+'/'+i+'/'+j)

        # Load dataset
        with open(final_path, 'r') as archive:
          text = archive.read()
        
        text_size = len(text.split('\n'))
        textSplit.extend(text.split('\n'))

        # Separate data
        for n in range(text_size):
          valSensor = textSplit[n+old_text_size].split('\t')
          if(valSensor!=['']): 

            # left knee data
            if(k == 1):
              for w in range(3):
                if(w == 0): 
                  if(X_max_left[0] < (float(valSensor[w+k])*(-1))): X_max_left[0] = (float(valSensor[w+k])*(-1)) #values to normalization
                  if(X_min_left[0] > (float(valSensor[w+k])*(-1))): X_min_left[0] = (float(valSensor[w+k])*(-1)) #values to normalization
                if(w == 1): 
                  if(X_max_left[1] < (float(valSensor[w+k])*(-1))): X_max_left[1] = (float(valSensor[w+k])*(-1)) #values to normalization
                  if(X_min_left[1] > (float(valSensor[w+k])*(-1))): X_min_left[1] = (float(valSensor[w+k])*(-1)) #values to normalization
                if(w == 2): 
                  if(X_max_left[2] < (float(valSensor[w+k])*(-1))): X_max_left[2] = (float(valSensor[w+k])*(-1)) #values to normalization
                  if(X_min_left[2] > (float(valSensor[w+k])*(-1))): X_min_left[2] = (float(valSensor[w+k])*(-1)) #values to normalization
                value.append([(float(valSensor[w+k])*(-1))])
              X_left.append(value) # Input
              if(y_max_left < (float(valSensor[7])*(-1))): y_max_left = (float(valSensor[7])*(-1)) #values to normalization
              if(y_min_left > (float(valSensor[7])*(-1))): y_min_left = (float(valSensor[7])*(-1)) #values to normalization
              y_left.append((float(valSensor[7])*(-1))) #desired output
              value = []

            # right knee data
            if(k == 4):
              for w in range(3):
                if(w == 0): 
                  if(X_max_right[0] < (float(valSensor[w+k])*(-1))): X_max_right[0] = (float(valSensor[w+k])*(-1)) #normalization
                  if(X_min_right[0] > (float(valSensor[w+k])*(-1))): X_min_right[0] = (float(valSensor[w+k])*(-1)) #normalization
                if(w == 1): 
                  if(X_max_right[1] < (float(valSensor[w+k])*(-1))): X_max_right[1] = (float(valSensor[w+k])*(-1)) #normalization
                  if(X_min_right[1] > (float(valSensor[w+k])*(-1))): X_min_right[1] = (float(valSensor[w+k])*(-1)) #normalization
                if(w == 2): 
                  if(X_max_right[2] < (float(valSensor[w+k])*(-1))): X_max_right[2] = (float(valSensor[w+k])*(-1)) #normalization
                  if(X_min_right[2] > (float(valSensor[w+k])*(-1))): X_min_right[2] = (float(valSensor[w+k])*(-1)) #normalization
                value.append([(float(valSensor[w+k])*(-1))])
              X_right.append(value) # Input
              if(y_max_right < (float(valSensor[7])*(-1))): y_max_right = (float(valSensor[7])*(-1)) #normalization
              if(y_min_right > (float(valSensor[7])*(-1))): y_min_right = (float(valSensor[7])*(-1)) #normalization
              y_right.append((float(valSensor[7])*(-1))) #desired output
              value = []

        old_text_size += text_size

# Creating arrays to recieved input and desired datas
X_all_left = np.ndarray(shape=(len(X_left), 3), dtype=float)
y_all_left = np.ndarray(shape=(len(y_left), 1), dtype=float)

X_all_right = np.ndarray(shape=(len(X_right), 3), dtype=float)
y_all_right = np.ndarray(shape=(len(y_right), 1), dtype=float)

# normalizing left knee data
for i in range(len(X_left)):
  for j in range(3):
    X_all_left[i][j] = ((float(X_left[i][j][0]) - X_min_left[j]) / (X_max_left[j] - X_min_left[j])) #input
  y_all_left[i] = ((float(y_left[i]) - y_min_left) / (y_max_left - y_min_left)) #desired output

# normalizing right knee data
for i in range(len(X_right)):
  for j in range(3):
    X_all_right[i][j] = ((float(X_right[i][j][0]) - X_min_right[j]) / (X_max_right[j] - X_min_right[j])) #input
  y_all_right[i] = ((float(y_right[i]) - y_min_right) / (y_max_right - y_min_right)) #desired output

#configure train data
X_left_train = X_all_left[0:int(len(X_all_left)*0.7)] #only 70% of dataset to train
y_left_train = y_all_left[0:int(len(y_all_left)*0.7)]

X_right_train = X_all_right[0:int(len(X_all_right)*0.7)] #only 70% of dataset to train
y_right_train = y_all_right[0:int(len(y_all_right)*0.7)]

#configure test data
X_left_test = X_all_left[int(len(X_all_left)*0.7)+1:] #only 20% of dataset to test
y_left_test = y_all_left[int(len(y_all_left)*0.7)+1:]

X_right_test = X_all_right[int(len(X_all_right)*0.7)+1:] #only 20% of dataset to test
y_right_test = y_all_right[int(len(y_all_right)*0.7)+1:]


'''
*
Create and train NN
*
'''

tf.random.set_seed(42) #set randon seed

#create left model
left_model = tf.keras.Sequential() #create model
left_model.add(tf.keras.Input(shape=(3,))) #create input layer
left_model.add(tf.keras.layers.Dense(10, activation='relu')) #create hidden layer
left_model.add(tf.keras.layers.Dense(1, activation='linear')) #create output layer

left_model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss='mse') #define optimizer and loss function

#create right model
right_model = tf.keras.Sequential() #create model
right_model.add(tf.keras.Input(shape=(3,))) #create input layer
right_model.add(tf.keras.layers.Dense(10, activation='relu')) #create hidden layer
right_model.add(tf.keras.layers.Dense(1, activation='linear')) #create output layer

right_model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss='mse') #define optimizer and loss function

#train models
print('TRAININNG LEFT MODEL\n')
left_model.fit(X_left_train, y_left_train, batch_size=750, epochs=70) #train left model
print('\n\n\nTRAINNIN RIGHT MODEL\n')
right_model.fit(X_left_train, y_left_train, batch_size=750, epochs=70) #train right model

#predict values
predicted_left = left_model.predict(X_left_test) 
predicted_right = right_model.predict(X_right_test)
