# -*- coding: utf-8 -*-
"""PPO.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1kxH3ALhP_WNiXIE7sO6y_mbvDZnUCuJV
"""

# "I certify that the code and data in this assignment were generated independently, using only the tools
# and resources defined in the course and that I did not receive any external help, coaching or contributions
# during the production of this work."

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical

class Actor(nn.Module):
  def __init__(self,state_space,action_space):
    super().__init__()
    self.fhl = nn.Linear(state_space,256)
    self.shl = nn.Linear(256,128)
    self.ol = nn.Linear(128,action_space)
    #self.double()
    return None
  
  def forward(self,x):
    x = F.relu(self.fhl(x))
    x = F.relu(self.shl(x))
    x = F.relu(self.ol(x))
    return x
  
  def update_weights(self,actor_loss,actor_optimizer):
    actor_optimizer.zero_grad()
    actor_loss.backward(retain_graph=True)
    actor_optimizer.step()

class Critic(nn.Module):
  def __init__(self,state_space):
    super().__init__()
    self.fhl = nn.Linear(state_space,256)
    self.shl = nn.Linear(256,128)
    self.ol = nn.Linear(128,1)
    #self.double()
    return None
  
  def forward(self,x):
    x = F.relu(self.fhl(x))
    x = F.relu(self.shl(x))
    x = F.relu(self.ol(x))
    return x
  
  def update_weights(self,critic_loss,critic_optimizer):
    critic_optimizer.zero_grad()
    critic_loss.backward(retain_graph=True)
    critic_optimizer.step()

class Q_Value(nn.Module):
  def __init__(self,state_space,action_space) -> None:
      super().__init__()
      self.fhl = nn.Linear(state_space+action_space+1,256)
      self.shl = nn.Linear(256,128)
      self.ol = nn.Linear(128,1)
      #self.double()
      return None
  
  def forward(self,x):
    x = F.relu(self.fhl(x))
    x = F.relu(self.shl(x))
    x = F.relu(self.ol(x))
    return x
  
  def update(self,q_loss,q_optimizer):
    q_optimizer.zero_grad()
    q_loss.backward(retain_graph=True)
    q_optimizer.step()