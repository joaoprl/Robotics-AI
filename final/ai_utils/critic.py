##
## implementation of the CRITIC network,
## along with target optimization
##

import tensorflow as tf
import keras.backend as K

from keras.initializers import VarianceScaling
from keras.layers import Dense, Flatten, Input, Lambda, Activation, add
from keras.models import Model
from keras.optimizers import Adam

H_LAYER1 = 300
H_LAYER2 = 400

class critic_network:
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
        self.helper, self.actions, self.state = self.build_network()
        self.target, self.target_actions, self.target_state = self.build_network()

        # policy update gradients
        self.action_gradients = tf.gradients(self.helper.output, self.actions)

        # set things up
        self.sess.run(tf.global_variables_initializer())

        print('~*~*~>critic network created!')
        self.target.summary()

    ## load our weights
    def load_weights(self, path):
        try:
            self.helper.load_weights(path+"/critic.h5")
            self.target.load_weights(path+"/critic.h5")

            print('Loading weights for critic network.')
        except:
            print('Uh oh! Couldn\'t load weights for critic network.')

    ## save our weights
    def save_weights(self, path):
        self.helper.save_weights(path+"/critic.h5", overwrite=True)

        print('Saving weights for critic network.')

    ## get our gradients for policy update
    def gradients(self, states, actions):
        return self.sess.run(self.action_gradients, feed_dict={
            self.state: states,
            self.actions: actions
        })[0]

    ##   apply training on our target network, to slowly converge with our
    ##  critic network
    def target_train(self):
        weights = self.helper.get_weights()
        target_weights = self.target.get_weights()

        for i in range(len(weights)):
            target_weights[i] = self.tau*weights[i] + \
                                            (1 - self.tau)*target_weights[i]

        self.target.set_weights(target_weights)

    ## build our network
    def build_network(self):
        S = Input(shape=[self.state_dim])
        w = Dense(H_LAYER1, activation='relu')(S)
        h1 = Dense(H_LAYER2, activation='linear')(w)

        A = Input(shape=[self.action_dim], name='action2')
        a = Dense(H_LAYER2, activation='linear')(A)

        h2 = add([h1, a])
        h3 = Dense(H_LAYER2, activation='relu')(h2)
        V = Dense(self.action_dim, activation='linear')(h3)

        model = Model(inputs=[S, A], outputs=V)
        model.compile(loss='mse', optimizer=Adam(lr=self.lr))

        return model, A, S

## testing...
if __name__ == "__main__":
    sess = tf.Session()
    K.set_session(sess)
    a = critic_network(sess, 25, 8)
