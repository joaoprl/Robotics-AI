# -*- coding: utf-8 -*-

### Implementation of the DDPG algorithm [1]. 
###
### [1] https://arxiv.org/pdf/1509.02971.pdf

from ai_utils import noise
from actor import actor_network
from critic import critic_network

from keras import backend as K
import tensorflow as tf

## TODO: -> debug main algorithm (networks should be good)
##       
##       -> integrate with robot environment

class ddpg:
    ##
    ## definition of our DDPG algorithm, default values are according to our
    ## reference [1].
    ##
    ##  -> action_dim:  total of actions;
    ##  -> state_dim:   total of states;
    ##  -> batch size:  batch size for sampling;
    ##
    ##  hyper parameters:
    ##  -> wd:      weight decay for Q;
    ##  -> gamma:   discount factor;
    ##  -> tau:     soft target updates
    ##  -> lra:     learning rate for ACTOR
    ##  -> lrc:     learning rate for CRITIC
    ##
    def __init__(self, action_dim, state_dim, batch_size=32, 
        wd=.01, gamma=.99, tau=.001, lra=.0001, lrc=.001, path='./aidata/'):

        self.path = path
        self.noise = noise.ou()

        self.batch_size = batch_size

        self.wd = wd
        self.gamma = gamma
        self.tau = tau

        self.lra = lra
        self.lrc = lrc

        self.action_dim = action_dim
        self.state_dim = state_dim

        # gpu usage
        # config = tf.ConfigProto()
        # config.gpu_options.allow_growth = True

        self.sess = tf.Session() # config=config
        K.set_session(self.sess)

    def load_weights(self, actor, critic):
        actor.load_weights(self.path)
        critic.load_weights(self.path)

    ## train our ddpg model
    def train(self, max_episodes, max_steps, buffer_size):
        # initialize actor and critic networks
        actor = actor_network(self.sess, self.state_dim, self.action_dim, 
            self.batch_size, self.tau, self.lra)

        critic = critic_network(self.sess, self.state_dim, self.action_dim, 
            self.batch_size, self.tau, self.lrc)

        # load weights
        load_weights(actor, critic)

        # initialize replay buffer R
        buff = replay_buffer(buffer_size)

        # helpers at training
        epsilon = 1
        explore_decay = 1.0/100000.

        for episode in range(max_episodes):
            ## initialize a random process for action exploration from our 
            ## VREP environment
            # self.simulator.restart()

            ## get initial state from our VREP environment
            # s_t = np.hstack(self.robot.get_state())

            total_reward = 0.

            for t in range(max_steps):
                ## select action according to our current policy
                a_t = np.zeros(action_dim)
                noise_t = np.zeros(action_dim)

                a_t_raw = actor.helper.predict(s_t)

                # apply exploration noise
                epsilon = max(epsilon-explore_decay, 0)
                noise_t = epsilon*self.noise(a_t_raw)

                a_t = a_t_raw + noise_t

                ## execute action and observe our new reward+state
                # new_s_t, r_t, done = self.robot.act(a_t)

                ## store transition at our buffer
                buff.store(s_t, a_t, r_t, new_s_t)

                ## sample random minibatch of transitions from R
                states, actions, rewards, new_states, dones = buff.get_batch()

                ## set y according to our choice
                target_q_values = critic.target.predict(new_states, 
                    actor.target.predict(new_states))

                y_t = r_t + dones*self.gamma*target_q_values

                ## update critics by minimizing loss
                loss = critic.helper.train([states, actions], y_t)

                ## update actor policy using the sampled policy gradient
                gradient_actions = actor.helper.predict(states)
                gradients = critic.gradients(states, gradient_actions)

                actor.train(states, gradients)

                ## update target networks
                actor.target_train()
                critic.target_train()

                s_t = new_s_t
                total_reward += r_t

                ## did we end our episode?
                if done:
                    break

            if episode % 5 is 0:
                self.actor.save_weights()
                self.critic.save_weights()

            print 'EPISODE: ' + episode
            print '\tTOTAL REWARD: ' + total_reward

    ## run our ddpg model
    def tick(self):
        # initialize actor and critic networks
        actor = actor_network(self.sess, self.state_dim, self.action_dim)
        critic = critic_network(self.sess, self.state_dim, self.action_dim)

        # load weights
        load_weights(actor, critic)

        for episode in range(max_episodes):
            ## initialize a random process for action exploration from our 
            ## VREP environment
            # self.simulator.restart()

            ## get initial state from our VREP environment
            # s_t = np.hstack(self.robot.get_state())

            total_reward = 0.
            for t in range(max_steps):
                ## select action according to our current policy
                a_t = actor.helper.predict(s_t)

                ## execute action and observe our new reward+state
                # new_s_t, r_t, done = self.robot.act(a_t)

                s_t = new_s_t
                total_reward += r_t

                ## did we end our episode?
                if done:
                    break

            print 'EPISODE: ' + episode
            print '\tTOTAL REWARD: ' + total_reward
