# myTeam.py
# ---------
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


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'QLearningAgent', second = 'QLearningAgent'):
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

  # The following line is an example only; feel free to change it.
  a = OffensiveAgent(index = firstIndex)
  b = DefensiveAgent( index = secondIndex)
  return [a, b]


class MyAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions
  """
 
  def __init__(self, index = 0, epsilon = .2, training = True, discount = .5, alpha = .2):

    self.index = index
    

    self.observationHistory = []
    self.depthLimit = 5
    

  
  def registerInitialState(self, gameState):
    #super(type(MyAgent)).registerInitialState(gameState)
    CaptureAgent.registerInitialState(self, gameState)
    self.red = gameState.isOnRedTeam(self.index)
    import distanceCalculator
    self.distancer = distanceCalculator.Distancer(gameState.data.layout)

    # comment this out to forgo maze distance computation and use manhattan distances
    self.distancer.getMazeDistances()
    self.redIndeces = []
    self.blueIndeces = []

    self.initialLocation = gameState.getAgentPosition(self.index)

    

    for i in range(4):
      if gameState.isOnRedTeam(i):
        self.redIndeces.append(i)
      else:
        self.blueIndeces.append(i)

    self.origNumFood = len(self.getFood(gameState).asList())
    #self.origNumBlueFood = len(gameState.getBlueFood().asList())

    self.origFoodLocs = self.getFood(gameState).asList()

    myTeam = self.getTeam(gameState)
    for i in myTeam:
      if not i == self.index:
        self.partnerIndex = i
  
  '''
  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """

    actions = gameState.getLegalActions(self.index)
    return toReturn
    '''


  def chooseAction(self, gameState):
      """
        Returns the minimax action using self.depth and self.evaluationFunction
      """
      "*** YOUR CODE HERE ***"
      #self.debugClear()
      toRet = self.alphaBeta(gameState,self.index, depth = 0, alpha = -100000000000, beta = 1000000000000, oldState = gameState)
      return toRet


  def alphaBeta (self, gameState, agentIndex = 0, depth = 0, alpha = -1000000000000, beta = 1000000000000, oldState = None, action = None):

    
    bestScore = None
    bestAction = None
    worstAction = None
    worstScore = None
    for a in gameState.getLegalActions(agentIndex):
      suc = gameState.generateSuccessor(agentIndex, a)
      if agentIndex in self.blueIndeces:
        val = self.maxValue(suc, agentIndex, depth, alpha, beta , oldState = oldState, actionToGetHere = a)
        if a == "Stop" and len( self.getFood(gameState).asList()) <= 2: 
          val -= 10
      else:
        
        val = self.minValue(suc, agentIndex, depth, alpha, beta , oldState = oldState, actionToGetHere = a)
        if a == "Stop" and len (self.getFood(gameState).asList()) <= 2: val += 10
      #min = self.minValue(suc, agentIndex, depth, alpha, beta)
      if bestScore is None or val > bestScore:
        bestScore = val
        bestAction = a
      if worstScore is None or val < worstScore:
        worstScore = val
        worstAction = a

    #print bestAction, worstAction, bestScore, worstScore
    #print bestScore, bestAction, agentIndex
    return bestAction if agentIndex in self.blueIndeces else worstAction

  def maxValue(self, gameState, agentIndex, depth, alpha, beta , oldState = None, actionToGetHere = None):
    #print depth
    #print depth
    if self.depthLimit <= depth:
      toRet = (self.evaluationFunction(gameState, agentIndex, oldState = oldState, action = actionToGetHere))
      return toRet

    bm = None
    bestScore = None
    v = -100000000000

    agentLocs = []
    for i in range (4): 
      agentLocs.append(gameState.getAgentPosition(i))

    locationOfThisGuy = agentLocs[agentIndex]
    if locationOfThisGuy is None:
      import random
      return self.evaluationFunction(gameState, agentIndex, action = actionToGetHere, oldState = oldState)
    #print ("HJERER")

    for move in gameState.getLegalActions(agentIndex):

        suc = gameState.generateSuccessor(agentIndex, move)
        
        
        if self.index == 3:
          nextIndex = 0
        else:
          nextIndex = self.index+1
        score = self.minValue(suc, nextIndex , depth +1, alpha, beta, actionToGetHere = move, oldState= oldState)
        # if move == "Stop":
        #   score -= 10000
        #print move, score
        v = max(v, score)

        if v > beta:
          if agentIndex == 1:
            print "PRUNING Because v is", v, "beta is", beta
          return (v)
        if (v > alpha):
          alpha = v
          bm = move
          bestScore = v
        #print v
    #print v
    #print bm, "BEST"
    return (v)


  def minValue(self, gameState, agentIndex, depth, alpha, beta , oldState = None, actionToGetHere = None):
    #print depth
    if self.depthLimit <= depth:
      return (self.evaluationFunction(gameState, agentIndex, oldState = oldState, action = actionToGetHere))

    bm = None
    v = 100000000000

    agentLocs = []
    for i in range (4): 
      agentLocs.append(gameState.getAgentPosition(i))
    #print agentLocs
    locationOfThisGuy = agentLocs[agentIndex]
    if locationOfThisGuy is None:
      import random
      return self.evaluationFunction(gameState, agentIndex, oldState = oldState, action= actionToGetHere)
    #print v, "1"
    #print agentLocs[agentIndex]
    for move in gameState.getLegalActions(agentIndex):
        suc = gameState.generateSuccessor(agentIndex, move)

        if self.index == 3:
          nextIndex = 0
        else:
          nextIndex = self.index+1

        score = self.maxValue(suc,  nextIndex ,depth + 1, alpha, beta , oldState = oldState, actionToGetHere = move)
        #print score, agentIndex
        # print score, "score"
        # print v, "2"
        # if move == "Stop":
        #   score += 10000
        v = min(v, score)

        # print ("HERE", v)
        # print v, "2.5"
        if v < alpha:
          return (v)
        if (v < beta):
          beta = v
          bm = move
        # print v, "3"

    return (v)


  def eval(self, features):
    return features * self.getWeights()

class OffensiveAgent(MyAgent):
  def evaluationFunction(self, gameState, index, action, oldState):
    """
    Returns a counter of features for the state
    """
    

    toRet = 0
    features = util.Counter()

    myFood = self.getFood(gameState).asList()
    previousStateFood = len(self.getFood(oldState).asList())
    myPos = gameState.getAgentState(self.index).getPosition()
    distToClosestFood = min([self.getMazeDistance(myPos, food) for food in myFood]+[10000])

    locationsOfGhosts = []

    enemies = self.getOpponents(gameState)

    for e in enemies:
      locationsOfGhosts.append (gameState.getAgentPosition(e))

    #generally stay far from ghosts
    totalDistanceToGhosts = 0
    for d in locationsOfGhosts:
      if d is not None:
        toAdd = self.getMazeDistance(gameState.getAgentPosition(self.index), d)
        totalDistanceToGhosts += toAdd

    distanceToCloseGhost = 7
    for e in enemies:
      if gameState.getAgentPosition(e) is not None:
        #skip thsi if first move and no previous state
        if not self.getPreviousObservation():
          continue
        #if the enemy is not pacman and we were pacman, dist is the distance from the dude
        if not gameState.getAgentState(e).isPacman and oldState.getAgentState(self.index).isPacman:
          #myPos = gameState.getAgentPosition(self.index)
          otherPos = gameState.getAgentPosition(e)
          distanceToCloseGhost = (self.getMazeDistance(myPos, otherPos))
          break
    
    numFoodBeingCarried = gameState.getAgentState(self.index).numCarrying
    oldFoodBeingCarried = oldState.getAgentState(self.index).numCarrying
    # if self.getPreviousObservation():
    #   oldFoodBeingCarried = self.getPreviousObservation().getAgentState(self.index).numCarrying
    # else:
    #   oldFoodBeingCarried = 0
    justAte = oldFoodBeingCarried < numFoodBeingCarried
    justAte = previousStateFood - len(myFood)
    #print justAte, myPos, oldFoodBeingCarried, numFoodBeingCarried, previousStateFood, len(myFood)

    score = gameState.getScore()*-1
    oldScore = oldState.getScore() * -1

    justScored = oldScore < score

    features['distToClosestFood'] = 1.0/(distToClosestFood+1)
    if justAte > 0:
      features['justAte'] = 1

    features['numFoodRemaining'] = 10.0/(len(myFood)+1)
    features['totalEaten'] = self.origNumFood - len(myFood)

    distanceToHome = self.getMazeDistance(myPos, self.initialLocation)
    if numFoodBeingCarried > 2 or oldFoodBeingCarried > 2:
      features['closeToHome'] = 1.0/(distanceToHome+1)

    #THIS SHIT NEEDS TO BE FIXEd
    if distanceToCloseGhost <4:
      features['closeToGhost'] = -1/(distanceToCloseGhost+1)
      features['closeToHome'] = 0.000010/(distanceToHome+1)
      features['distToClosestFood'] = 0

    
    toRet = self.eval(features)
    if action is "Stop":
      toRet -=100
    

    # self.debugDraw(myPos, [1,0,0])
    
    # self.debugDraw([l for l in locationsOfGhosts if l is not None], [0,1,0])
    
    

    if gameState.isOnRedTeam(self.index):
      toRet *= -1
    #print toRet, features
    return toRet



  def getWeights(self):
    return {'closeToGhost':100000000000, 'totalEaten': 100, 'distToClosestFood': .001, 'justAte': 10000, "closeToHome": 1000000000}
    #return {'justAte': 100, 'foodEaten': 1000, 'x': 6, "distToHome":6, "score": 990, "tooClose":-10000000000, "closestFood":.00045, "amountOfFoodToEat" : 90}
  
class DefensiveAgent(MyAgent):
  def evaluationFunction(self, gameState, index, action = None, oldState = None):
    features = util.Counter()

    enemies = self.getOpponents(gameState)
    dist = 0
    myPos = gameState.getAgentPosition(self.index)
    for e in enemies:
      if gameState.getAgentPosition(e) is not None:


        #FIX TJHIS
        if gameState.getAgentState(e).isPacman:
          #myPos = gameState.getAgentPosition(self.index)
          otherPos = gameState.getAgentPosition(e)
          dist = self.getMazeDistance(myPos, otherPos)
    

    
    features["distToThem"] = dist
    
    averageXOfMyFood = 0
    averageYOfMyFood = 0

    foodImDefending = self.getFoodYouAreDefending(gameState).asList()

    #print foodImDefending
    for f in foodImDefending:
      averageXOfMyFood += f[0]
      averageYOfMyFood += f[1]
    
    closest = None
    closestDist = None
    avgFood = (averageXOfMyFood/(len(foodImDefending)+1), averageYOfMyFood/(len(foodImDefending)+1))
    for f in foodImDefending:
      d = util.manhattanDistance(f, avgFood)
      if closest is None or d< closestDist:
        closest = f
        closestDist = d
    

    #FIX THIS
    
    distFromAvgLocationOfFood = self.getMazeDistance(myPos, closest)

    features['distanceToMyFood'] = distFromAvgLocationOfFood

    toRet = self.eval(features)
    if gameState.isOnRedTeam(self.index):
      toRet *= -1
    
    return toRet

  def getWeights(self):
    weights = {"distToThem": -50, "distanceToMyFood": -5}
    return weights
