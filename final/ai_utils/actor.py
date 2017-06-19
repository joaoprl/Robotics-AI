
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
        self.helper, self.weights, self.state = self.build_network()
        self.target, self.target_weights, self.target_state = self.build_network()

        self.action_gradients = tf.placeholder(tf.float32, [None, self.action_dim])
        self.params_grad = tf.gradients(self.model.output, self.weights, 
            -self.action_gradients)

        grads = zip(self.params_grad, self.weights)

        self.optimize = tf.train.AdamOptimizer(self.lr).apply_gradients(grads)

        # set things up
        self.sess.run(tf.initialize_all_variables())

    ## get our gradients for policy update
    def gradients(self, states, grads):
        return self.sess.run(self.action_gradients, feed_dict={
            self.state: states
            self.action_gradients: grads
            })[0]

    ##   apply training on our target network, to slowly converge with our
    ##  actor network
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
        h0 = Dense(LAYER1, activation='relu')(S)
        h1 = Dense(LAYER2, activation='relu')(h0)
        V = Dense(self.action_dim, activation='linear')(h3)

        # set each output individually
        # Steering = Dense(1, activation='tanh', init=lambda shape, 
        #     name: normal(shape, scale=1e-4, name=name))(h1)  
        # Acceleration = Dense(1, activation='sigmoid', init=lambda shape, 
        #     name: normal(shape, scale=1e-4, name=name))(h1)   
        # Brake = Dense(1, activation='sigmoid', init=lambda shape, 
        #     name: normal(shape, scale=1e-4, name=name))(h1)
        #
        # V = merge([Steering, Acceleration, Brake], mode='concat')

        model = Model(input=S, output=V)

        return model, model.trainable_weights, S
