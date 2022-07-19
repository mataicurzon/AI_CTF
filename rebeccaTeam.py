# baselineTeam.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


# baselineTeam.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html
import capture
import pacman
from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='OffensiveReflexAgent', second='DefensiveReflexAgent'):
    """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):
    """
  A base class for reflex agents that chooses score-maximizing actions
  """

    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)
        print('start', self.start)
        print('start[0]', self.start[0])

        CaptureAgent.registerInitialState(self, gameState)
        self.counter = 0
        self.counter2 = 0
        self.isStop = False
        self.totalFood = len(self.getFood(gameState).asList())
        self.opponentIndices = self.getOpponents(gameState)
        # number of opponents
        self.numOpponents = len(self.opponentIndices)
        # empty dict {} in the beginning for opponent positions
        self.opponentPositions = util.Counter()

        # self.totalDist = self.getMazeDistance(self.start, self.opponentStart)

        # find initial opponent positions
        for opponent in self.opponentIndices:
            self.opponentPositions[opponent] = gameState.getInitialAgentPosition(opponent)
            # self.opponentStart = gameState.getInitialAgentPosition(opponent)

        # print('oppenentPos', self.opponentPositions[1][0])

    """def betterEvaluationFunction(self, currentGameState):
       
        currentPos = currentGameState.getPacmanPosition()
        currentFood = currentGameState.getFood()
        foodList = currentFood.asList()
        distances = []  # list of distances from Pacman vs food
        ghostStates = currentGameState.getGhostStates()
        scaredTimes = [ghostState.scaredTimer for ghostState in ghostStates]
        ghostPos = currentGameState.getGhostPositions()
        tempScore = []  # list of temporary scores in the form [(score, (x,y) )]

        for x, y in foodList:  # find the Manhattan distances from the food vs Pacman position
            distances.append(distanceCalculator.manhattanDistance(currentPos, (x, y)))

        if len(distances) >= 0:
            avgDistance = (sum(distances) + 1) / (len(distances) + 1)

        constant = 50  # 20

        for index, scaredTime in enumerate(scaredTimes):  # iterate through ghosts and new scared times
            if scaredTime == 0:  # if ghosts are not scared
                if distanceCalculator.manhattanDistance(currentPos, ghostPos[index]) < 2:  # if ghosts are nearby
                    tempScore.append((-999999, ghostPos[index]))
                else:
                    tempScore.append(((constant / avgDistance) + currentGameState.getScore(), currentPos))
            else:  # if ghosts are scared
                tempScore.append(((constant / avgDistance) + currentGameState.getScore(), currentPos))

        tempScore.sort()
        length = len(tempScore)
        return tempScore[length - 1][0]"""

    def chooseAction(self, gameState):
        """
    Picks among the actions with the highest Q(s,a).
    """
        actions = gameState.getLegalActions(self.index)

        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()
        values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        if foodLeft <= 2:
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)

                if dist < bestDist:
                    bestAction = action
                    bestDist = dist

            return bestAction

        return random.choice(bestActions)

    def getSuccessor(self, gameState, action):
        """
    Finds the next successor which is a grid position (location tuple).
    """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def evaluate(self, gameState, action):
        """
    Computes a linear combination of features and feature weights
    """
        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        return features * weights

    def getFeatures(self, gameState, action):
        """
    Returns a counter of features for the state
    """
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        features['successorScore'] = self.getScore(successor)

        return features

    def getWeights(self, gameState, action):
        """
    Normally, weights do not depend on the gamestate.  They can be either
    a counter or a dictionary.
    """
        return {'successorScore': 1.0}


class OffensiveReflexAgent(ReflexCaptureAgent):
    """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """

    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)

        # print("counter:", self.counter)

        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()
        values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        #print("scared timer", gameState.getAgentState(self.index).scaredTimer)

        if self.totalFood > foodLeft:
            self.counter += 1
            print("totalfood", self.totalFood)
            print('counter', self.counter)
            self.totalFood = foodLeft

            print('foodleft', foodLeft)

        if self.counter >= 3 or foodLeft <= 2:
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)
                nearest = nearestPoint(pos2)

                if dist < bestDist:
                    if gameState.hasFood(nearest[0], nearest[1]):
                        dist = self.getMazeDistance(self.start, nearest)
                        print('reached nearest')
                    bestAction = action
                    bestDist = dist

                if not gameState.getAgentState(self.index).isPacman:
                    self.counter = 0
                    print('counter reset')

                """if not gameState.isOnRedTeam(self.index):

                    if pos2[0] >= (self.start[0] / 2 + 1) or pos2 == self.start:
                        self.counter = 0
                        print('counter reset')

                else:
                    if pos2[0] <= (self.opponentPositions[1][0] / 2) or pos2 == self.start:
                        self.counter = 0
                        print('counter reset', self.counter)"""

            return bestAction

        a = random.choice(bestActions)
        #print('action', a)

        if a == 'Stop':
            if not gameState.isOnRedTeam(self.index):
                for action in gameState.getLegalActions(self.index):
                    if action == 'East':
                        return action
            else:
                for action in gameState.getLegalActions(self.index):
                    if action == 'West':
                        return action

        # print('isStop', self.isStop)

        return a

    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        foodList = self.getFood(successor).asList()
        features['successorScore'] = -len(foodList)  # self.getScore(successor)

        # Compute distance to the nearest food

        if len(foodList) > 0:  # This should always be True,  but better safe than sorry
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        if self.counter >= 3:
            features['foodEaten'] = self.getScore(successor) * -1

        return features

    def getWeights(self, gameState, action):
        return {'successorScore': 100, 'distanceToFood': -1, 'foodEaten': 100}


class DefensiveReflexAgent(ReflexCaptureAgent):
    """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        #print("counter", self.counter)

        # print("counter:", self.counter)

        # You can profile your evaluation time by uncommenting these lines
        # start = time.time()
        values = [self.evaluate(gameState, a) for a in actions]
        # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        foodLeft = len(self.getFood(gameState).asList())

        # print("scared timer", gameState.getAgentState(self.index).scaredTimer)

        if self.isStop is True or gameState.getAgentState(self.index).scaredTimer > 0:


            if self.totalFood > foodLeft:
                self.counter += 1
                # print("totalfood", self.totalFood)
                # print('counter', self.counter)
                self.totalFood = foodLeft

                # print('foodleft', foodLeft)

        if self.counter >= 3 or foodLeft <= 2:
            bestDist = 9999
            for action in actions:
                successor = self.getSuccessor(gameState, action)
                pos2 = successor.getAgentPosition(self.index)
                dist = self.getMazeDistance(self.start, pos2)
                nearest = nearestPoint(pos2)

                if dist < bestDist:
                    if gameState.hasFood(nearest[0], nearest[1]):
                        dist = self.getMazeDistance(self.start, nearest)
                        print('reached nearest')
                    bestAction = action
                    bestDist = dist

                if not gameState.getAgentState(self.index).isPacman:
                    self.counter = 0
                    print('counter reset')

            return bestAction

        return random.choice(bestActions)



    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        if self.isStop is True or gameState.getAgentState(self.index).scaredTimer > 0:
            foodList = self.getFood(successor).asList()
            features['successorScore'] = -len(foodList)  # self.getScore(successor)

            if len(foodList) > 0:  # This should always be True,  but better safe than sorry
                myPos = successor.getAgentState(self.index).getPosition()
                minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
                features['distanceToFood'] = minDistance

            return features
        else:

            myState = successor.getAgentState(self.index)
            myPos = myState.getPosition()

            # Computes whether we're on defense (1) or offense (0)
            features['onDefense'] = 1
            if myState.isPacman: features['onDefense'] = 0

            # Computes distance to invaders we can see
            enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
            invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
            features['numInvaders'] = len(invaders)
            if len(invaders) > 0:
                dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
                features['invaderDistance'] = min(dists)

            if action == Directions.STOP: features['stop'] = 1
            rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
            if action == rev: features['reverse'] = 1

            return features

    def getWeights(self, gameState, action):
        if self.isStop is True or gameState.getAgentState(self.index).scaredTimer > 0:
            return {'successorScore': 100, 'distanceToFood': -1, 'foodEaten': 100}

        else:
            return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}

    """class AlphaBetaAgent(ReflexCaptureAgent):
        

        def getAction(self, gameState):
            

            def alphaBeta(currentState, depth, agentIndex, alpha, beta):
                # check if terminal state or depth is exceeded
                if depth >= self.depth or len(currentState.getLegalActions(agentIndex)) < 1:
                    return self.evaluationFunction(currentState), None

                if agentIndex == 0:  # if Pacman, find max
                    bestValue = float("-inf")  # v = negative infinity
                    bestAction = None
                    legalActions = currentState.getLegalActions(agentIndex)  # Pacman's legal actions
                    # find max value, action for Pacman
                    for action in legalActions:
                        successorState = currentState.generateSuccessor(agentIndex, action)
                        tempValue, tempAction = alphaBeta(successorState, depth, agentIndex + 1, alpha, beta)
                        if tempValue > bestValue:
                            bestValue, bestAction = tempValue, action
                            alpha = max(alpha, bestValue)

                        if bestValue > beta:
                            return bestValue, bestAction
                    return bestValue, bestAction

                else:  # find min for ghosts
                    bestValue = float("inf")  # v = positive infinity
                    bestAction = None  # agent's best action to take
                    legalActions = currentState.getLegalActions(agentIndex)  # agent's(ghost's) legal actions
                    # find min value, action for each ghost
                    for action in legalActions:
                        if agentIndex + 1 < currentState.getNumAgents():  # check to stay in bounds of number of agents
                            successorState = currentState.generateSuccessor(agentIndex, action)
                            tempValue, tempAction = alphaBeta(successorState, depth, agentIndex + 1, alpha, beta)
                            if tempValue < bestValue:
                                bestValue, bestAction = tempValue, action
                                beta = min(beta, bestValue)

                            if bestValue < alpha:
                                return bestValue, bestAction

                        else:  # if end of ghosts (choose Pacman)
                            successorState = currentState.generateSuccessor(agentIndex, action)
                            tempValue, tempAction = alphaBeta(successorState, depth + 1, 0, alpha, beta)
                            if tempValue < bestValue:
                                bestValue, bestAction = tempValue, action
                                beta = min(beta, bestValue)

                            if bestValue < alpha:
                                return bestValue, bestAction
                    return bestValue, bestAction

            # Actually finding best move -- call minimax on Pacman
            val, act = alphaBeta(gameState, 0, 0, float("-inf"), float("inf"))
            return act"""
