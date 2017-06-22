##
## implementation of the ACTOR network, 
## along with target optimization
##

import tensorflow as tf
import keras.backend as K

from keras.initializers import VarianceScaling
from keras.layers import Dense, Flatten, Input, Lambda, Activation
from keras.models import Model

H_LAYER1 = 300
H_LAYER2 = 400

class actor_network:
    def __init__(self, sess, state_dim, action_dim, batch_size=32, tau=.001, 
        lr=.001):
        self.sess = sess

        self.state_dim = state_dim
        self.action_dim = action_dim

        self.batch_size = batch_size
        self.tau = tau
        self.lr = lr

        K.set_session(self.sess)

        # create models
        self.helper, self.weights, self.state = self.build_network()
        self.target, self.target_weights, self.target_state = self.build_network()

        # policy update gradients
        self.action_gradients = tf.placeholder(tf.float32, [None, self.action_dim])
        self.params_grad = tf.gradients(self.helper.output, self.weights, 
            -self.action_gradients)

        grads = zip(self.params_grad, self.weights)

        self.optimize = tf.train.AdamOptimizer(self.lr).apply_gradients(grads)

        # set things up
        self.sess.run(tf.global_variables_initializer())

        print '~*~*~>actor network created!'
        self.target.summary()

    ## load our weights
    def load_weights(self, path):
        try:
            self.helper.load_weights(path+"/actor.h5")
            self.target.load_weights(path+"/actor.h5")

            print 'Loading weights for actor network.'
        except:
            print 'Uh oh! Couldn\'t load weights for actor network.'

    ## save our weights
    def save_weights(self, path):
        self.helper.save_weights(path+"/actor.h5", overwrite=True)

        print 'Saving weights for actor network.'

    ## get our gradients for policy update
    def gradients(self, states, grads):
        return self.sess.run(self.action_gradients, feed_dict={
            self.state: states,
            self.action_gradients: grads
        })[0]

    ##   apply training on our target network, to slowly converge with our
    ##  actor network
    def target_train():
        weights = self.helper.get_weights()
        target_weights = self.target.get_weights()

        for i in range(len(weights)):
            target_weights[i] = self.tau*weights[i] + \
                                            (1 - self.tau)*target_weights[i]

        self.target.set_weights(target_weights)

    ## build our network
    def build_network(self):
        S = Input(shape=[self.state_dim])
        h0 = Dense(H_LAYER1, activation='relu')(S)
        h1 = Dense(H_LAYER2, activation='relu')(h0)
        V = Dense(self.action_dim, activation='tanh', kernel_initializer=\
            lambda shape: VarianceScaling(scale=1e-4)(shape))(h1)

        model = Model(inputs=S, outputs=V)

        return model, model.trainable_weights, S

## testing...
if __name__ == "__main__":
    sess = tf.Session()
    K.set_session(sess)
    a = actor_network(sess, 25, 8)
