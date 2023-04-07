#save models TensorFlow
left_model.save('/content/left_model')
right_model.save('/content/right_model')

'''
*
TensorFlow -> TensorFlow Lite
*
'''
# Convert left the model
converter_left = tf.lite.TFLiteConverter.from_saved_model('/content/left_model') # path to the SavedModel directory
tflite_left_model = converter_left.convert()

# Save the model.
with open('/content/left_model.tflite', 'wb') as f:
  f.write(tflite_left_model)

# Convert right the model
converter_right = tf.lite.TFLiteConverter.from_saved_model('/content/right_model') # path to the SavedModel directory
tflite_right_model = converter_right.convert()

# Save the model.
with open('/content/right_model.tflite', 'wb') as f:
  f.write(tflite_right_model)
  
'''
*
Encode the Model in an Arduino Header File
*
'''
!echo "const unsigned char model[] = {" > /content/left_model.h
!cat left_model.tflite | xxd -i              >> /content/left_model.h
!echo "};"                              >> /content/left_model.h

import os
model_h_size = os.path.getsize("left_model.h")
print(f"Header file, left_model.h, is {model_h_size:,} bytes.")
print("\nOpen the side panel (refresh if needed). Double click model.h to download the file.")


!echo "const unsigned char model[] = {" > /content/right_model.h
!cat right_model.tflite | xxd -i              >> /content/right_model.h
!echo "};"                              >> /content/right_model.h

import os
model_h_size = os.path.getsize("right_model.h")
print(f"Header file, right_model.h, is {model_h_size:,} bytes.")
print("\nOpen the side panel (refresh if needed). Double click model.h to download the file.")
