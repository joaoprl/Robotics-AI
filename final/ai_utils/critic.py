
import tensorflow as tf
import keras.backend as K

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
        self.sess.run(tf.initialize_all_variables())

    ## get our gradients for policy update
    def gradients(self, states, actions):
        return self.sess.run(self.action_gradients, feed_dict={
            self.state: states
            self.actions: actions
            })[0]

    ##   apply training on our target network, to slowly converge with our
    ##  critic network
    def target_train():
        weights = self.helper.get_weights()
        target_weights = self.target.get_weights()

        for i in range(len(weights)):
            target_weights[i] = self.tau*weights[i] + 
                                            (1 - self.tau)*target_weights[i]

        self.target.set_weights(target_weights)

    ## build our network
    def build_network(self):
        S = Input(shape=[self.state_size])
        w = Dense(H_LAYER1, activation='relu')(S)
        h1 = Dense(H_LAYER2, activation='linear')(w)

        A = Input(shape=[self.action_dim], name='action2')
        a = Dense(H_LAYER2, activation='linear')(A) 

        h2 = merge([h1, a], mode='sum')
        h3 = Dense(H_LAYER2, activation='relu')(h2)
        V = Dense(self.action_dim, activation='linear')(h3)

        model = Model(input=[S, A], output=V)
        model.compile(loss='mse', optimizer=Adam(lr=self.lr))

        return model, A, S
