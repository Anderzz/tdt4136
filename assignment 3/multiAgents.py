# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util
import math

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.
        """
        def minValue(state, depth, index):
            v= math.inf
            #get the legal actions for the agent corresponding to index
            legalActions = state.getLegalActions(index)
            #if we have reached a terminal state
            #return its utility
            if  state.isLose() or state.isWin():
                return self.evaluationFunction(state)
            #else continue    
            else:
                for action in legalActions:
                    #check who's turn it is
                    if index == state.getNumAgents() -1:
                        v= min(v, maxValue(state.generateSuccessor(index, action), depth + 1))
                    else:
                        v= min(v, minValue(state.generateSuccessor(index, action), depth, index + 1))
            return v


        def maxValue(state, depth):
            v= -math.inf
            #get the agents legal actions
            legalActions = state.getLegalActions(0)
            if depth > self.depth:
                return self.evaluationFunction(state)
            if state.isLose() or state.isWin():
                return self.evaluationFunction(state)
                
            else:
                for action in legalActions:
                    v = max(v, minValue(state.generateSuccessor(0, action), depth, 1))
            return v
        
        return max(gameState.getLegalActions(0), key=lambda action: minValue(gameState.generateSuccessor(0, action), 1, 1))

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        alpha = -math.inf
        beta  =  math.inf

        def minValue(state, depth, index, alpha, beta):
            v= math.inf
            #get the legal actions for the agent corresponding to index
            legalActions = state.getLegalActions(index)
            #if we are have reached a terminal state
            #return its utility
            if  state.isLose() or state.isWin():
                return self.evaluationFunction(state)
            #else continue    
            else:
                for action in legalActions:
                    #check who's turn it is
                    if index == state.getNumAgents() -1:
                        v= min(v, maxValue(state.generateSuccessor(index, action), depth + 1, alpha, beta))
                    else:
                        v= min(v, minValue(state.generateSuccessor(index, action), depth, index + 1, alpha, beta))
                    if v< alpha:
                        return v
                    beta = min(v, beta)
            return v



        def maxValue(state, depth, alpha, beta, give_action=False):
            v= -math.inf
            #get the agents legal actions
            legalActions = state.getLegalActions(0)
            if depth > self.depth:
                return self.evaluationFunction(state)
            #if we have reached a terminal state
            #return its utility
            if state.isLose() or state.isWin():
                return self.evaluationFunction(state)
                
            else:
                for action in legalActions:
                    v2 = minValue(state.generateSuccessor(0, action), depth, 1, alpha, beta)
                    if v2 > v:
                        new_action=action
                        v = v2
                    if v > beta:
                        return v
                    alpha = max(v, alpha)
                if give_action:
                    return new_action
            return v
        
        return maxValue(gameState, 1, alpha, beta, True)

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction
