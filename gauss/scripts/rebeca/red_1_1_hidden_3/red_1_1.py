# -*- coding: utf-8 -*-
"""
Created on Tue Apr 24 20:02:07 2018

@author: rebeca fernandez niederacher
"""
import tensorflow as tf
import w_definition
import datos_entrada
import numpy as np
from sklearn.metrics import accuracy_score

model_path = '/media/rebeca/COMPARTIDA/GIERM_TFG/codigo_tensorflow/codigos_py/git_jose/jamrepo/gauss/scripts/save_model'

n_csv = w_definition.uav 
n_obs = w_definition.obs
num_label = datos_entrada.label_len
input_len = int(1*datos_entrada.simulation_len)
num_features = datos_entrada.features_len #position relativa cada UAV+ veloidad relativa cada UAV + distancia goal + distancia obs
"""
DECLARAMOS VARIABLES DE ENTRADA A LA RED
"""
batch_data =[]
batch_labels = []


train_size = int(0.7*input_len)

valid_size = int(0.15*input_len)
"""
DEFINE THE FLAGS USEABLE FROM THE COMMAND LINE. YA INICIALIZADOS A ESE VALOR!
"""
tf.app.flags.DEFINE_integer('num_epochs', 400,
                            'Number of examples to separate from the training '
                            'data for the validation set.')
                            
tf.app.flags.DEFINE_float('learning_rate', 0.006, 'Initial learning rate.')

tf.app.flags.DEFINE_integer('train_size', train_size, 'set of training data')

tf.app.flags.DEFINE_integer('valid_size', valid_size, 'set of validation data')

tf.app.flags.DEFINE_integer('BATCH_SIZE', 3, 'Must divide evenly into the dataset sizes.')

tf.app.flags.DEFINE_integer('n_hidden_1', 30, 'Nuumber of neuron in layer 1')

tf.app.flags.DEFINE_integer('n_hidden_2', 30, 'Nuumber of neuron in layer 2')

tf.app.flags.DEFINE_integer('n_hidden_3', 30, 'Nuumber of neuron in layer 3')

FLAGS = tf.app.flags.FLAGS

"""
MATRICES DE ENTRADA Y SALIDA COMPLETAS CREADAS EN DATOS_ENTRADA.PY
"""
def randomize(dataset, labels):
    permutation = np.random.permutation(len(labels))
    shuffled_dataset = dataset[permutation, :]
    shuffled_labels = labels[permutation, :]
    return shuffled_dataset, shuffled_labels
    
data_input = datos_entrada.input_matrix
labels_input = datos_entrada.label 

data_input = np.matrix(data_input, np.float32)
labels_input = np.matrix(labels_input, np.float32)

"""
IF THE SIZE OF THE TRAINING IS BIIGER THAN THE COMPLETE SIMULATION SET, IT IS NOT POSSIBLE
TO COMPUTE THE NETWORK. IT IS SET TO "FALSE".
"""
error = False

num_epochs = FLAGS.num_epochs
learning_rate = FLAGS.learning_rate
BATCH_SIZE = FLAGS.BATCH_SIZE
train_size = FLAGS.train_size
valid_size = FLAGS.valid_size
n_hidden_1 =FLAGS.n_hidden_1
n_hidden_2 =FLAGS.n_hidden_2
n_hidden_3 =FLAGS.n_hidden_3

if (train_size>input_len):
    error = True
    print ("ERROR!")
    print ()
    print ("TRAINING SIZE BIGGER THAN THE COMPLETE SIMULATION SET. YOU SHOULD MAKE IT SMALLER")
    print ()
    print ("training size:")
    print (train_size)
 
"""
    DIVIDIMOS EL DATA SET Y LA MATRIZ DE SALIDA, EN UN SET DE TRAINING Y OTRO DE TEST
""" 

dataset, labels = randomize(data_input,labels_input)

train_data =  dataset[0:train_size, :]
train_labels = labels[0:train_size, :]
    
valid_data = dataset[train_size: valid_size+train_size, : ]
valid_labels = labels[train_size: valid_size+train_size, :]
    
test_data= dataset[valid_size+train_size : input_len, :]
test_labels= labels[valid_size+train_size : input_len, :]

"""
FOR REGULARIZATION
"""
beta1=0.005
beta2 = 0.001 #0.005 no0 da tan buenos resultados
beta3 =0.005

graph = tf.Graph()
with graph.as_default():
    """
    FOR THE TRAINING DATA, WE USE A PLACEHOLDER THAT WILL BE FED AT RUN TIME
    WITH A TRAINING MINIBATCH
    """
    tf_train_dataset = tf.placeholder(tf.float32, shape=(None, num_features))
    tf_train_labels = tf.placeholder(tf.float32, shape=(None, num_label))
    tf_valid_dataset = tf.constant(valid_data)
    tf_test_dataset = tf.constant(test_data)
 
    """
    WEIGHTS AND BIASES INITIALIZATION----> sttdev = sqrt(2/n_hidden_previous) 
    https://discussions.udacity.com/t/problem-3-3-dropout-does-not-improve-test-accuarcy/46286/15
    """
    dev_1 =  0.7
    dev_2 = np.sqrt(2/n_hidden_1) # with 22 neurons da 0.3 tambien
    dev_3 = np.sqrt(2/n_hidden_2)
    dev_out = np.sqrt(2/n_hidden_3)
    
    weights = {
        'h1': tf.Variable(tf.random_normal([num_features, n_hidden_1], stddev = dev_1, dtype= np.float32)),
        'h2': tf.Variable(tf.random_normal([n_hidden_1, n_hidden_2], stddev = dev_2, dtype= np.float32)),
        'h3': tf.Variable(tf.random_normal([n_hidden_2, n_hidden_3], stddev = dev_3, dtype= np.float32)),
        'out': tf.Variable(tf.random_normal([n_hidden_3, num_label], stddev = dev_out, dtype= np.float32))
        }
   
    biases = {
        'b1': tf.Variable(tf.zeros([n_hidden_1], dtype= np.float32)),
        'b2': tf.Variable(tf.zeros([n_hidden_2],dtype= np.float32)),
        'b3': tf.Variable(tf.zeros([n_hidden_3],dtype= np.float32)),
        'out': tf.Variable(tf.zeros([num_label],dtype= np.float32))
    }
    
    """
    MULTILAYER ARCHITECTURE. NOW WITHOUT DROPOUT
    """
    def multilayer_perceptron(x):
        # Hidden fully connected layer with n_hidden_1 neurons
        layer_1 = tf.nn.tanh(tf.add(tf.matmul(x, weights['h1']), biases['b1']))
        # Hidden fully connected layer with n_hidden_2 neurons
        layer_2 = tf.nn.tanh(tf.add(tf.matmul(layer_1, weights['h2']), biases['b2']))
        #DROPOUT
        #dropout = tf.nn.dropout(layer_2, dropout)
         # Hidden fully connected layer with n_hidden_3 neurons
        layer_3 = tf.nn.tanh(tf.add(tf.matmul(layer_2, weights['h3']), biases['b3']))
        # Output fully connected layer with a neuron for each class
        out_layer = tf.nn.tanh(tf.matmul(layer_3, weights['out']) + biases['out'])
        
        return (out_layer)
  
    # Training computation.
    logits = multilayer_perceptron(tf_train_dataset)

    """
    REGULARIZATION (no mucha diferencia)
    """

    loss_regu = beta1*(tf.nn.l2_loss( weights['h1']))+ beta2*( tf.nn.l2_loss(weights['h2'])+tf.nn.l2_loss(weights['h3'])+tf.nn.l2_loss(weights['out']))
    """
    COST with regularization
    """
    loss = tf.losses.mean_squared_error(labels=tf_train_labels, predictions =logits)  + loss_regu
    """
    LEARNING RATE DECAY    
    """
    
    #global_step = tf.Variable(0)  # count the number of steps taken.
    #learning_rate = tf.train.exponential_decay(0.005, global_step, 10000, 1, True)
    """
    OPTIMIZER
    """
    # Optimizer. #GradientDescentOptimizer
    #optimizer = tf.train.MomentumOptimizer(learning_rate,0.5).minimize(loss)
    optimizer = tf.train.GradientDescentOptimizer(learning_rate).minimize(loss)
    """
    PREDICTIONS FOR THE TRAINING, VALIDATION AND TEST DATA
    """
    train_prediction = logits
    valid_prediction = multilayer_perceptron(tf_valid_dataset)
    test_prediction = multilayer_perceptron(tf_test_dataset)
    
# 'saver' op to save and restore all the variables
    saver = tf.train.Saver()
    """
    ACCURACY FUNCTION
    """
def accuracy(predictions, labels):
    percent = accuracy_score(np.argmax(labels,1), np.argmax(predictions,1))*100
    return percent


"""
SESSION RUN
"""
with tf.Session(graph=graph) as session:
    tf.global_variables_initializer().run()
    print("Initialized")
    for step in range((num_epochs * train_size // BATCH_SIZE)):
        # OFFSET IN THE TRAINING
        offset = (step * BATCH_SIZE) % (train_size-BATCH_SIZE)
        # GGENERATE MINIBATCHES
        batch_data = train_data[offset:(offset + BATCH_SIZE), :]
        batch_labels = train_labels[offset:(offset + BATCH_SIZE), :]
        # Prepare a dictionary telling the session where to feed the minibatch.
        # The key of the dictionary is the placeholder node of the graph to be fed,
        # and the value is the numpy array to feed to it.
        feed_dict = {tf_train_dataset: batch_data, tf_train_labels: batch_labels}
        _, l, l_regu, predictions = session.run([optimizer, loss, loss_regu, train_prediction], feed_dict=feed_dict)
        saver.save(session, "world_nn", global_Step = 2000)
        if (step % 2000 == 0):
            print("Minibatch loss at step %d: %f" % (step, l))
            print ("loss regularization at step %d: %f" % (step, l_regu))
            print("Minibatch accuracy: %.1f%%" % accuracy(predictions, batch_labels))
            print("Validation accuracy: %.1f%%" % accuracy(valid_prediction.eval(), valid_labels))
    print("Test accuracy: %.1f%%" % accuracy(test_prediction.eval(), test_labels))
    save_path = saver.save(session, model_path)
   

