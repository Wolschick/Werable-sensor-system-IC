from torch import nn
import onnx
from onnx_tf.backend import prepare
import tensorflow as tf


'''
Pytorch to ONNX
'''

pytorch_left_model = mlp_left
example_left_input = X_test_left_tensor
ONNX_LEFT_PATH = "./onnx_left_model.onnx"

pytorch_right_model = mlp_right
example_right_input = X_test_right_tensor
ONNX_RIGHT_PATH = "./onnx_right_model.onnx"

torch.save(pytorch_left_model.state_dict(), 'left_model.pt')
torch.save(pytorch_right_model, 'right_model.pt')

pytorch_left_model.load_state_dict(torch.load('/content/left_model.pt'))

# save left model
torch.onnx.export(
    model = pytorch_left_model,
    args = example_left_input, 
    f = ONNX_LEFT_PATH, # where should it be saved
    verbose=True,
    export_params=True,
    do_constant_folding=True,  # fold constant values for optimization
    # do_constant_folding=True,   # fold constant values for optimization
    input_names=['input'],
    output_names=['output']
)

onnx_left_model = onnx.load(ONNX_LEFT_PATH)
onnx.checker.check_model(onnx_left_model)

# save right model
torch.onnx.export(
    model = pytorch_right_model,
    args = example_right_input, 
    f = ONNX_RIGHT_PATH, # where should it be saved
    verbose=True,
    export_params=True,
    do_constant_folding=True,  # fold constant values for optimization
    # do_constant_folding=True,   # fold constant values for optimization
    input_names=['input'],
    output_names=['output']
)

onnx_right_model = onnx.load(ONNX_RIGHT_PATH)
onnx.checker.check_model(onnx_right_model)


'''
ONNX to TensorFlow
'''

TF_LEFT_PATH = "./tf_left_model.tf" # where the representation of tensorflow model will be stored
ONNX_LEFT_PATH = "./onnx_left_model.onnx" # path to my existing ONNX model
onnx_left_model = onnx.load(ONNX_LEFT_PATH)  # load onnx model

TF_RIGHT_PATH = "./tf_right_model.tf" # where the representation of tensorflow model will be stored
ONNX_RIGHT_PATH = "./onnx_right_model.onnx" # path to my existing ONNX model
onnx_right_model = onnx.load(ONNX_RIGHT_PATH)  # load onnx model

# prepare function converts an ONNX model to an internel representation
# of the computational graph called TensorflowRep and returns
# the converted representation.
tf_left_rep = prepare(onnx_left_model)  # creating TensorflowRep object
tf_right_rep = prepare(onnx_right_model)  # creating TensorflowRep object

# export_graph function obtains the graph proto corresponding to the ONNX
# model associated with the backend representation and serializes
# to a protobuf file.
tf_left_rep.export_graph(TF_LEFT_PATH)
tf_right_rep.export_graph(TF_RIGHT_PATH)


'''
TensorFlow to TensorFlow Lite
'''

TFLITE_LEFT_PATH = "./tflite_left_model.tflite"
TF_LEFT_PATH = '/content/tf_left_model.tf'

TFLITE_RIGHT_PATH = "./tflite_right_model.tflite"
TF_RIGHT_PATH = '/content/tf_right_model.tf'

# left values
converter_left = tf.lite.TFLiteConverter.from_saved_model('tf_left_model.tf')
converter_left.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS,
    tf.lite.OpsSet.SELECT_TF_OPS,
]

tflite_left_model = converter_left.convert()

# right values
converter_right = tf.lite.TFLiteConverter.from_saved_model('tf_right_model.tf')
converter_right.target_spec.supported_ops = [
    tf.lite.OpsSet.TFLITE_BUILTINS,
    tf.lite.OpsSet.SELECT_TF_OPS,
]

tflite_right_model = converter_right.convert()



#save the model
with open('left_model.tflite', 'wb') as f:
  f.write(tflite_left_model)

with open('right_model.tflite', 'wb') as f:
  f.write(tflite_right_model)
