# -*- coding: utf-8 -*-

### Implementation of the DDPG algorithm [1]. 
###
### [1] https://arxiv.org/pdf/1509.02971.pdf

from ai_utils import noise, replay_buffer as rb
from ai_utils.actor import actor_network
from ai_utils.critic import critic_network

from keras import backend as K
import tensorflow as tf
import numpy as np

## TODO: -> integrate with robot environment

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
    def __init__(self, robot, state_dim, action_dim, 
        batch_size=32, wd=.01, gamma=.99, tau=.001, lra=.0001, lrc=.001, 
        path='./aidata/'):

        self.robot = robot
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

    ##########
    ## act!
    ##########

    ## train our ddpg model
    def train(self, max_episodes, max_steps, buffer_size=1000000):
        # initialize actor and critic networks
        actor = actor_network(self.sess, self.state_dim, self.action_dim, 
            self.batch_size, self.tau, self.lra)

        critic = critic_network(self.sess, self.state_dim, self.action_dim, 
            self.batch_size, self.tau, self.lrc)

        # load weights
        self.load_weights(actor, critic)

        # initialize replay buffer R
        buff = rb.replay_buffer(buffer_size, self.batch_size)

        # helpers at training
        epsilon = 1
        explore_decay = 1.0/100000.

        for episode in range(max_episodes):
            ## initialize a random process for action exploration from our 
            ## VREP environment
            self.robot.reset_robot()

            ## get initial state from our VREP environment
            s_t = self.bake(self.robot.get_state())

            total_reward = 0.
            for t in range(max_steps):
                ## select action according to our current policy
                a_t = np.zeros(self.action_dim)
                noise_t = np.zeros(self.action_dim)

                a_t_raw = actor.helper.predict(s_t)

                # apply exploration noise
                epsilon = max(epsilon-explore_decay, 0)
                noise_t = epsilon*self.noise.apply(a_t_raw)

                a_t = a_t_raw + noise_t

                ## execute action and observe our new reward+state
                new_s_t, r_t, done = self.robot.act(a_t[0])
                new_s_t = self.bake(new_s_t)

                ## store transition at our buffer
                buff.store(s_t, a_t, r_t, new_s_t, done)

                ## sample random minibatch of transitions from buffer
                states, actions, rewards, new_states, dones = buff.get_batch()

                ## set y according to our choice
                target_q_values = critic.target.predict([new_states, \
                    actor.target.predict(new_states)])

                y_t = r_t + dones*self.gamma*target_q_values

                ## update critics by minimizing loss
                loss = critic.helper.train_on_batch([states, actions], y_t)

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
                actor.save_weights(self.path)
                critic.save_weights(self.path)

            print '*********************************************'
            print 'EPISODE: ' + str(episode)
            print '\tTOTAL REWARD: ' + str(total_reward)
            print '*********************************************'

    ## run our ddpg model
    def run(self, max_episodes, max_steps):
        # initialize actor and critic networks
        actor = actor_network(self.sess, self.state_dim, self.action_dim)
        critic = critic_network(self.sess, self.state_dim, self.action_dim)

        # load weights
        self.load_weights(actor, critic)

        for episode in range(max_episodes):
            ## initialize a random process for action exploration from our 
            ## VREP environment
            self.robot.reset_robot()

            ## get initial state from our VREP environment
            s_t = self.bake(self.robot.get_state())

            total_reward = 0.
            for t in range(max_steps):
                ## select action according to our current policy
                a_t = actor.helper.predict(s_t)

                ## execute action and observe our new reward+state
                new_s_t, r_t, done = self.robot.act(a_t[0])

                s_t = self.bake(new_s_t)
                total_reward += r_t

                ## did we end our episode?
                if done:
                    break

            print '*********************************************'
            print '\tEPISODE: ' + str(episode)
            print '\tTOTAL REWARD: ' + str(total_reward)
            print '*********************************************'

    ##########
    ## helpers
    ##########

    ## bake our input
    def bake(self, arr):
        return np.reshape(arr, (1, len(arr))).astype(np.float32)

## testing...
class robot:
    def __init__(self, s_dim, a_dim):
        self.s = s_dim
        self.a = a_dim
       
    def reset_robot(self):
        return

    def get_state(self):
        return [1]*25

    def act(self, actions):
        print '\t\t-> Actions to be executed: ' + str(actions)
        return [1]*self.s, 1, False

if __name__ == "__main__":
    r = robot(25, 8)
    ai = ddpg(r, 25, 8)
    ai.train(10, 10)
