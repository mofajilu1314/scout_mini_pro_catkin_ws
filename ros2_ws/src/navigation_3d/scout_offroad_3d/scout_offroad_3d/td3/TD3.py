# Adapted from https://github.com/sfujim/TD3/blob/master/TD3.py (MIT)
# Copyright (c) 2020 Scott Fujimoto — see LICENSE_sfujim_TD3
# Paper: https://arxiv.org/abs/1802.09477
#
# Scout change: max_action may be a scalar or a length-action_dim vector
# so (v, ω) can use different scales (m/s, rad/s).
# Scout: hidden_dim configurable (default 256 Fujimoto; overnight uses 512).

import copy

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _to_max_action_tensor(max_action, action_dim):
    arr = np.asarray(max_action, dtype=np.float32).reshape(-1)
    if arr.size == 1:
        arr = np.full((action_dim,), float(arr[0]), dtype=np.float32)
    if arr.size != action_dim:
        raise ValueError("max_action size %d != action_dim %d" % (arr.size, action_dim))
    return torch.FloatTensor(arr).to(device)


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action, hidden_dim=256):
        super(Actor, self).__init__()
        h = int(hidden_dim)
        self.l1 = nn.Linear(state_dim, h)
        self.l2 = nn.Linear(h, h)
        self.l3 = nn.Linear(h, action_dim)
        self.max_action = _to_max_action_tensor(max_action, action_dim)

    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()
        h = int(hidden_dim)
        self.l1 = nn.Linear(state_dim + action_dim, h)
        self.l2 = nn.Linear(h, h)
        self.l3 = nn.Linear(h, 1)
        self.l4 = nn.Linear(state_dim + action_dim, h)
        self.l5 = nn.Linear(h, h)
        self.l6 = nn.Linear(h, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        q2 = F.relu(self.l4(sa))
        q2 = F.relu(self.l5(q2))
        q2 = self.l6(q2)
        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], 1)
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        return q1


class TD3(object):
    def __init__(
        self,
        state_dim,
        action_dim,
        max_action,
        discount=0.99,
        tau=0.005,
        policy_noise=0.2,
        noise_clip=0.5,
        policy_freq=2,
        actor_lr=3e-4,
        critic_lr=3e-4,
        hidden_dim=256,
    ):
        self.hidden_dim = int(hidden_dim)
        self.max_action = _to_max_action_tensor(max_action, action_dim)
        self.actor = Actor(state_dim, action_dim, max_action, hidden_dim=self.hidden_dim).to(
            device
        )
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)

        self.critic = Critic(state_dim, action_dim, hidden_dim=self.hidden_dim).to(device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr)

        self.discount = discount
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq
        self.total_it = 0

    def select_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def train(self, replay_buffer, batch_size=256):
        self.total_it += 1
        state, action, next_state, reward, not_done = replay_buffer.sample(batch_size)

        with torch.no_grad():
            noise = (
                torch.randn_like(action) * self.policy_noise * self.max_action
            ).clamp(-self.noise_clip * self.max_action, self.noise_clip * self.max_action)
            next_action = (self.actor_target(next_state) + noise).clamp(
                -self.max_action, self.max_action
            )
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + not_done * self.discount * target_Q

        current_Q1, current_Q2 = self.critic(state, action)
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        if self.total_it % self.policy_freq == 0:
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            for param, target_param in zip(
                self.critic.parameters(), self.critic_target.parameters()
            ):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )
            for param, target_param in zip(
                self.actor.parameters(), self.actor_target.parameters()
            ):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )

    def save(self, filename):
        torch.save(self.critic.state_dict(), filename + "_critic")
        torch.save(self.critic_optimizer.state_dict(), filename + "_critic_optimizer")
        torch.save(self.actor.state_dict(), filename + "_actor")
        torch.save(self.actor_optimizer.state_dict(), filename + "_actor_optimizer")

    def load(self, filename, map_location=None):
        loc = map_location or device
        self.critic.load_state_dict(torch.load(filename + "_critic", map_location=loc))
        self.critic_optimizer.load_state_dict(
            torch.load(filename + "_critic_optimizer", map_location=loc)
        )
        self.critic_target = copy.deepcopy(self.critic)
        self.actor.load_state_dict(torch.load(filename + "_actor", map_location=loc))
        self.actor_optimizer.load_state_dict(
            torch.load(filename + "_actor_optimizer", map_location=loc)
        )
        self.actor_target = copy.deepcopy(self.actor)
