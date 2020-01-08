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


import random

import util
from game import Agent, Directions  # noqa
from util import manhattanDistance  # noqa


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
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices)  # Pick randomly among the best

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
        GHOST_COEF = 10.0
        FOOD_COEF = 10.0
        # find the distance for the ghost that closet to the Pacman
        mini_ghost = float("inf")
        for i in range(len(newGhostStates)):
            d = manhattanDistance(newGhostStates[i].getPosition(), newPos)

            if d <= 1 and newScaredTimes[i] == 0:
                return -float("inf")
            if d < mini_ghost:
                mini_ghost = d

        #find the distance for the food that closet to the Pacman
        mini_food = float("inf")
        for food_pos in newFood.asList():
            d = manhattanDistance(food_pos, newPos)
            if d < 1:
                return float("inf")
            if d < mini_food:
                mini_food = d

        return successorGameState.getScore() + (FOOD_COEF / mini_food) + (mini_ghost / GHOST_COEF)


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

    def __init__(self, evalFn="scoreEvaluationFunction", depth="2"):
        self.index = 0  # Pacman is always agent index 0
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

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        "*** YOUR CODE HERE ***"
        return self.minimax(gameState, 1, self.index, None)[0]


    def minimax(self, gameState, depth, agentIndex, action):
        """
        Recursive helper function for minimax algorithm
        """
        if (depth >= self.depth + 1 and agentIndex == 0) or gameState.isWin() or gameState.isLose():
            return (action, self.evaluationFunction(gameState))
        else:
            min_action = max_action = Directions.STOP
            min_score = float("inf")
            max_score = -float("inf")
            actions = gameState.getLegalActions(agentIndex)
            next_depth = depth
            if agentIndex == gameState.getNumAgents() - 1:
                next_depth = depth + 1
            for act in actions:
                next_game_state = gameState.generateSuccessor(agentIndex, act)
                action_score = self.minimax(next_game_state, next_depth, (agentIndex + 1)% gameState.getNumAgents(), act)
                if action_score[1] < min_score:
                    min_score = action_score[1]
                    min_action = act
                if action_score[1] > max_score:
                    max_score = action_score[1]
                    max_action = act
            if agentIndex == 0:
                return (max_action, max_score)
            else:
                return (min_action, min_score)


class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        return self.alphaBetaPruning(-float("inf"), float("inf"), gameState, 1,self.index, None)[0]


    def alphaBetaPruning(self, alpha, beta, gameState, depth, agentIndex, action):
        """
        Recursive helper function for alpha beta pruning.
        """
        if (depth >= self.depth + 1 and agentIndex == 0) or gameState.isWin() or gameState.isLose():
            return (action, self.evaluationFunction(gameState))
        else:
            min_action = max_action = Directions.STOP
            min_score = float("inf")
            max_score = -float("inf")
            actions = gameState.getLegalActions(agentIndex)
            nextDepth = depth
            if agentIndex == gameState.getNumAgents() - 1:
                nextDepth = depth + 1
            for act in actions:
                nextGameState = gameState.generateSuccessor(agentIndex, act)
                action_score = self.alphaBetaPruning\
                    (alpha, beta, nextGameState, nextDepth, (agentIndex + 1)% gameState.getNumAgents(), act)
                if action_score[1] < min_score:
                    min_score = action_score[1]
                    min_action = act
                if action_score[1] > max_score:
                    max_score = action_score[1]
                    max_action = act
                if agentIndex == 0:
                    if alpha < action_score[1]:
                        alpha = action_score[1]
                else:
                    if beta > action_score[1]:
                        beta = action_score[1]
                if alpha >= beta:
                    break
            if agentIndex == 0:
                return (max_action, max_score)
            else:
                return (min_action, min_score)


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
        return self.expectimax(gameState, 1, self.index)[0]


    def expectimax(self, gameState, depth, agentIndex):
        """
        Recursive helper function for Expectimax algorithm
        """
        if (depth >= self.depth + 1 and agentIndex == 0) or gameState.isWin() or gameState.isLose():
            return (None, self.evaluationFunction(gameState))
        else:
            max_action = Directions.STOP
            sum_score = 0
            max_score = -float("inf")
            actions = gameState.getLegalActions(agentIndex)
            nextDepth = depth
            if agentIndex == gameState.getNumAgents() - 1:
                nextDepth = depth + 1
            for act in actions:
                nextGameState = gameState.generateSuccessor(agentIndex, act)
                action_score = \
                    self.expectimax(nextGameState, nextDepth, (agentIndex + 1)% gameState.getNumAgents())
                sum_score += action_score[1]
                if action_score[1] > max_score:
                    max_score = action_score[1]
                    max_action = act
            if agentIndex == 0:
                return (max_action, max_score)
            else:
                return (None, sum_score/len(actions))




def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: find out the closet food position, reciprocal values of all distances between ghosts and pacman and
      reciprocal values of all distance between scared ghosts and pacman.
      Give these three kinds of values different coefficient as weight in order to make the reciprocals of these values
      more meaningful and influential.
      For the food and normal ghosts, the coefficients for them are both 10.0 as what we did in the Q1.
      For the scared ghosts, we give a larger coefficient 100.0 because eating an scared ghost can get a reward of 200
      comparing the reward of eating a food which is 10.
    """
    "*** YOUR CODE HERE ***"
    curPos = currentGameState.getPacmanPosition()
    curFood = currentGameState.getFood()
    curGhostStates = currentGameState.getGhostStates()
    curScaredTimes = [ghostState.scaredTimer for ghostState in curGhostStates]

    FOOD_COEF = 10.0
    GHOST_COEF = 10.0
    SCARED_GHOST_COEF = 100.0

    if currentGameState.isWin() or currentGameState.getNumFood() == 0:
        return float('inf')
    if currentGameState.hasWall(curPos[0], curPos[1]) or currentGameState.isLose():
        return -float('inf')
    mini_ghost = float("inf")
    scared_ghost_value = 0
    ghost_value = 0
    mini_food = float("inf")
    for i in range(len(curGhostStates)):
        d = manhattanDistance(curGhostStates[i].getPosition(), curPos)
        if d == 0:
            return -float("inf")
        if curScaredTimes[i] > 0 :
            scared_ghost_value += SCARED_GHOST_COEF/d
        else:
            ghost_value += GHOST_COEF/d
            if d < mini_ghost:
                mini_ghost = d

    # find the distance for the food that closet to the Pacman
    for food_pos in curFood.asList():
        d = manhattanDistance(food_pos, curPos)
        if d < mini_food:
            mini_food = d

    return currentGameState.getScore() + FOOD_COEF / mini_food + scared_ghost_value - ghost_value

# Abbreviation
better = betterEvaluationFunction
