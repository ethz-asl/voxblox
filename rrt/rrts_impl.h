#ifndef NBVP_VOXBLOX_RRT_RTTS_IMPL_H_
#define NBVP_VOXBLOX_RRT_RTTS_IMPL_H_

#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>

#include "nbvp_voxblox/rrt/rrts.h"

namespace nbvp_voxblox {

template <class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>::Vertex() {
  state = NULL;
  parent = NULL;
  trajFromParent = NULL;

  costFromParent = 0.0;
  costFromRoot = 0.0;
}

template <class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>::~Vertex() {
  if (state) delete state;
  parent = NULL;
  if (trajFromParent) delete trajFromParent;
  children.clear();
}

template <class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>::Vertex(
    const Vertex<State, Trajectory, System>& vertexIn) {
  if (vertexIn.state)
    state = new State(vertexIn.getState());
  else
    state = NULL;
  parent = vertexIn.parent;
  for (typename std::set<Vertex<State, Trajectory, System>*>::const_iterator
           iter = vertexIn.children.begin();
       iter != vertexIn.children.end(); iter++)
    children.insert(*iter);
  costFromParent = vertexIn.costFromParent;
  costFromRoot = vertexIn.costFromRoot;
  if (vertexIn.trajFromParent)
    trajFromParent = new Trajectory(*(vertexIn.trajFromParent));
  else
    trajFromParent = NULL;
}

// int Vertex::setState (const State &stateIn) {
//   *state = stateIn;
//   return 1;
// }

template <class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>::Planner() {
  gamma = 1.0;

  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  kdtree = NULL;

  root = NULL;

  numVertices = 0;

  system = NULL;
}

template <class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>::~Planner() {
  // Delete the kdtree structure
  if (kdtree) {
    kd_clear(kdtree);
    kd_free(kdtree);
  }

  // Delete all the vertices
  for (typename std::list<Vertex<State, Trajectory, System>*>::iterator iter =
           listVertices.begin();
       iter != listVertices.end(); iter++)
    delete *iter;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::insertIntoKdtree(
    Vertex<State, Trajectory, System>& vertexIn) {
  double* stateKey = new double[numDimensions];
  system->getStateKey(*(vertexIn.state), stateKey);
  kd_insert(kdtree, stateKey, &vertexIn);
  delete[] stateKey;

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::getNearestVertex(
    State& stateIn, Vertex<State, Trajectory, System>*& vertexPointerOut) {
  // Get the state key for the query state
  double* stateKey = new double[numDimensions];
  system->getStateKey(stateIn, stateKey);

  // Search the kdtree for the nearest vertex
  KdRes* kdres = kd_nearest(kdtree, stateKey);
  if (kd_res_end(kdres)) vertexPointerOut = NULL;
  vertexPointerOut =
      (Vertex<State, Trajectory, System>*)kd_res_item_data(kdres);

  // Clear up the memory
  delete[] stateKey;
  kd_res_free(kdres);

  // Return a non-positive number if any errors
  if (vertexPointerOut == NULL) return 0;

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::getNearVertices(
    State& stateIn,
    std::vector<Vertex<State, Trajectory, System>*>& vectorNearVerticesOut) {
  // Get the state key for the query state
  double* stateKey = new double[numDimensions];
  system->getStateKey(stateIn, stateKey);

  // Compute the ball radius
  double ballRadius = gamma * pow(log((double)(numVertices + 1.0)) /
                                      ((double)(numVertices + 1.0)),
                                  1.0 / ((double)numDimensions));

  // Search kdtree for the set of near vertices
  KdRes* kdres = kd_nearest_range(kdtree, stateKey, ballRadius);
  delete[] stateKey;

  // Create the vector data structure for storing the results
  int numNearVertices = kd_res_size(kdres);
  if (numNearVertices == 0) {
    vectorNearVerticesOut.clear();
    return 1;
  }
  vectorNearVerticesOut.resize(numNearVertices);

  // Place pointers to the near vertices into the vector
  int i = 0;
  kd_res_rewind(kdres);
  while (!kd_res_end(kdres)) {
    Vertex<State, Trajectory, System>* vertexCurr =
        (Vertex<State, Trajectory, System>*)kd_res_item_data(kdres);
    vectorNearVerticesOut[i] = vertexCurr;
    kd_res_next(kdres);
    i++;
  }

  // Free temporary memory
  kd_res_free(kdres);

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::checkUpdateBestVertex(
    Vertex<State, Trajectory, System>& vertexIn) {
  if (system->isReachingTarget(vertexIn.getState())) {
    double costCurr = vertexIn.getCost();
    if ((lowerBoundVertex == NULL) ||
        ((lowerBoundVertex != NULL) && (costCurr < lowerBoundCost))) {
      lowerBoundVertex = &vertexIn;
      lowerBoundCost = costCurr;
    }
  }

  return 1;
}

template <class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>*
RRTstar::Planner<State, Trajectory, System>::insertTrajectory(
    Vertex<State, Trajectory, System>& vertexStartIn,
    Trajectory& trajectoryIn) {
  // Check for admissible cost-to-go
  if (lowerBoundVertex != NULL) {
    double costToGo = system->evaluateCostToGo(trajectoryIn.getEndState());
    if (costToGo >= 0.0)
      if (lowerBoundCost < vertexStartIn.getCost() + costToGo) return NULL;
  }

  // Create a new end vertex
  Vertex<State, Trajectory, System>* vertexNew =
      new Vertex<State, Trajectory, System>;
  vertexNew->state = new State;
  vertexNew->parent = NULL;
  vertexNew->getState() = trajectoryIn.getEndState();
  insertIntoKdtree(*vertexNew);
  this->listVertices.push_front(vertexNew);
  this->numVertices++;

  // Insert the trajectory between the start and end vertices
  insertTrajectory(vertexStartIn, trajectoryIn, *vertexNew);

  return vertexNew;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::insertTrajectory(
    Vertex<State, Trajectory, System>& vertexStartIn, Trajectory& trajectoryIn,
    Vertex<State, Trajectory, System>& vertexEndIn) {
  // Update the costs
  vertexEndIn.costFromParent = trajectoryIn.evaluateCost();
  vertexEndIn.costFromRoot =
      vertexStartIn.costFromRoot + vertexEndIn.costFromParent;
  checkUpdateBestVertex(vertexEndIn);

  // Update the trajectory between the two vertices
  if (vertexEndIn.trajFromParent) delete vertexEndIn.trajFromParent;
  vertexEndIn.trajFromParent = new Trajectory(trajectoryIn);

  // Update the parent to the end vertex
  if (vertexEndIn.parent) vertexEndIn.parent->children.erase(&vertexEndIn);
  vertexEndIn.parent = &vertexStartIn;

  // Add the end vertex to the set of chilren
  vertexStartIn.children.insert(&vertexEndIn);

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::setSystem(System& systemIn) {
  if (system) delete system;

  system = &systemIn;

  numDimensions = system->getNumDimensions();

  // Delete all the vertices
  for (typename std::list<Vertex<State, Trajectory, System>*>::iterator iter =
           listVertices.begin();
       iter != listVertices.end(); iter++)
    delete *iter;
  numVertices = 0;
  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  // Clear the kdtree
  if (kdtree) {
    kd_clear(kdtree);
    kd_free(kdtree);
  }
  kdtree = kd_create(numDimensions);

  // Initialize the root vertex
  root = new Vertex<State, Trajectory, System>;
  root->state = new State(system->getRootState());
  root->costFromParent = 0.0;
  root->costFromRoot = 0.0;
  root->trajFromParent = NULL;

  return 1;
}

template <class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>&
RRTstar::Planner<State, Trajectory, System>::getRootVertex() {
  return *root;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::initialize() {
  // If there is no system, then return failure
  if (!system) return 0;

  // Backup the root
  Vertex<State, Trajectory, System>* rootBackup = NULL;
  if (root) rootBackup = new Vertex<State, Trajectory, System>(*root);

  // Delete all the vertices
  for (typename std::list<Vertex<State, Trajectory, System>*>::iterator iter =
           listVertices.begin();
       iter != listVertices.end(); iter++)
    delete *iter;
  listVertices.clear();
  numVertices = 0;
  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  // Clear the kdtree
  if (kdtree) {
    kd_clear(kdtree);
    kd_free(kdtree);
  }
  kdtree = kd_create(system->getNumDimensions());

  // Initialize the variables
  numDimensions = system->getNumDimensions();
  root = rootBackup;
  if (root) {
    listVertices.push_back(root);
    insertIntoKdtree(*root);
    numVertices++;
  }
  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::setGamma(double gammaIn) {
  if (gammaIn < 0.0) return 0;

  gamma = gammaIn;

  return 1;
}

template <class State, class Trajectory, class System>
int compareVertexCostPairs(
    std::pair<RRTstar::Vertex<State, Trajectory, System>*, double> i,
    std::pair<RRTstar::Vertex<State, Trajectory, System>*, double> j) {
  return (i.second < j.second);
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::findBestParent(
    State& stateIn,
    std::vector<Vertex<State, Trajectory, System>*>& vectorNearVerticesIn,
    Vertex<State, Trajectory, System>*& vertexBest, Trajectory& trajectoryOut,
    bool& exactConnection) {
  // Compute the cost of extension for each near vertex
  int numNearVertices = vectorNearVerticesIn.size();

  std::vector<std::pair<Vertex<State, Trajectory, System>*, double> >
      vectorVertexCostPairs(numNearVertices);

  int i = 0;
  for (typename std::vector<Vertex<State, Trajectory, System>*>::iterator iter =
           vectorNearVerticesIn.begin();
       iter != vectorNearVerticesIn.end(); iter++) {
    vectorVertexCostPairs[i].first = *iter;
    exactConnection = false;
    double trajCost = system->evaluateExtensionCost(*((*iter)->state), stateIn,
                                                    exactConnection);
    vectorVertexCostPairs[i].second = (*iter)->costFromRoot + trajCost;
    i++;
  }

  // Sort vertices according to cost
  std::sort(vectorVertexCostPairs.begin(), vectorVertexCostPairs.end(),
            compareVertexCostPairs<State, Trajectory, System>);

  // Try out each extension according to increasing cost
  i = 0;
  bool connectionEstablished = false;
  for (typename std::vector<std::pair<Vertex<State, Trajectory, System>*,
                                      double> >::iterator iter =
           vectorVertexCostPairs.begin();
       iter != vectorVertexCostPairs.end(); iter++) {
    Vertex<State, Trajectory, System>* vertexCurr = iter->first;

    // Extend the current vertex towards stateIn (and this time check for
    // collision with obstacles)
    exactConnection = false;
    if (system->extendTo(*(vertexCurr->state), stateIn, trajectoryOut,
                         exactConnection) > 0) {
      vertexBest = vertexCurr;
      connectionEstablished = true;
      break;
    }
  }

  // Return success if a connection was established
  if (connectionEstablished) return 1;

  // If the connection could not be established then return zero
  return 0;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::updateBranchCost(
    Vertex<State, Trajectory, System>& vertexIn, int depth) {
  // Update the cost for each children
  for (typename std::set<Vertex<State, Trajectory, System>*>::iterator iter =
           vertexIn.children.begin();
       iter != vertexIn.children.end(); iter++) {
    Vertex<State, Trajectory, System>& vertex = **iter;

    vertex.costFromRoot = vertexIn.costFromRoot + vertex.costFromParent;

    checkUpdateBestVertex(vertex);

    updateBranchCost(vertex, depth + 1);
  }

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::rewireVertices(
    Vertex<State, Trajectory, System>& vertexNew,
    std::vector<Vertex<State, Trajectory, System>*>& vectorNearVertices) {
  // Repeat for all vertices in the set of near vertices
  for (typename std::vector<Vertex<State, Trajectory, System>*>::iterator iter =
           vectorNearVertices.begin();
       iter != vectorNearVertices.end(); iter++) {
    Vertex<State, Trajectory, System>& vertexCurr = **iter;

    // Check whether the extension results in an exact connection
    bool exactConnection = false;
    double costCurr = system->evaluateExtensionCost(
        *(vertexNew.state), *(vertexCurr.state), exactConnection);
    if ((exactConnection == false) || (costCurr < 0)) continue;

    // Check whether the cost of the extension is smaller than current cost
    double totalCost = vertexNew.costFromRoot + costCurr;
    if (totalCost < vertexCurr.costFromRoot - 0.001) {
      // Compute the extension (checking for collision)
      Trajectory trajectory;
      if (system->extendTo(*(vertexNew.state), *(vertexCurr.state), trajectory,
                           exactConnection) <= 0)
        continue;

      // Insert the new trajectory to the tree by rewiring
      insertTrajectory(vertexNew, trajectory, vertexCurr);

      // Update the cost of all vertices in the rewired branch
      updateBranchCost(vertexCurr, 0);
    }
  }

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::iteration() {
  // 1. Sample a new state
  State stateRandom;
  system->sampleState(stateRandom);

  // 2. Compute the set of all near vertices
  std::vector<Vertex<State, Trajectory, System>*> vectorNearVertices;
  getNearVertices(stateRandom, vectorNearVertices);

  // 3. Find the best parent and extend from that parent
  Vertex<State, Trajectory, System>* vertexParent = NULL;
  Trajectory trajectory;
  bool exactConnection = false;

  if (vectorNearVertices.size() == 0) {
    // 3.a Extend the nearest
    if (getNearestVertex(stateRandom, vertexParent) <= 0) return 0;
    if (system->extendTo(vertexParent->getState(), stateRandom, trajectory,
                         exactConnection) <= 0)
      return 0;
  } else {
    // 3.b Extend the best parent within the near vertices
    if (findBestParent(stateRandom, vectorNearVertices, vertexParent,
                       trajectory, exactConnection) <= 0)
      return 0;
  }

  // 3.c add the trajectory from the best parent to the tree
  Vertex<State, Trajectory, System>* vertexNew =
      insertTrajectory(*vertexParent, trajectory);
  if (vertexNew == NULL) return 0;

  // 4. Rewire the tree
  if (vectorNearVertices.size() > 0)
    rewireVertices(*vertexNew, vectorNearVertices);

  return 1;
}

template <class State, class Trajectory, class System>
int RRTstar::Planner<State, Trajectory, System>::getBestTrajectory(
    std::list<double*>& trajectoryOut) {
  if (lowerBoundVertex == NULL) return 0;

  Vertex<State, Trajectory, System>* vertexCurr = lowerBoundVertex;

  while (vertexCurr) {
    State& stateCurr = vertexCurr->getState();

    double* stateArrCurr = new double[2];
    stateArrCurr[0] = stateCurr[0];
    stateArrCurr[1] = stateCurr[1];

    trajectoryOut.push_front(stateArrCurr);

    Vertex<State, Trajectory, System>& vertexParent = vertexCurr->getParent();

    if (&vertexParent != NULL) {
      State& stateParent = vertexParent.getState();

      std::list<double*> trajectory;
      system->getTrajectory(stateParent, stateCurr, trajectory);

      trajectory.reverse();
      for (std::list<double*>::iterator iter = trajectory.begin();
           iter != trajectory.end(); iter++) {
        double* stateArrFromParentCurr = *iter;

        stateArrCurr = new double[2];
        stateArrCurr[0] = stateArrFromParentCurr[0];
        stateArrCurr[1] = stateArrFromParentCurr[1];

        trajectoryOut.push_front(stateArrCurr);

        delete[] stateArrFromParentCurr;
      }
    }

    vertexCurr = &vertexParent;
  }

  return 1;
}

};  // namespace nbvp_voxblox

#endif  // NBVP_VOXBLOX_RRT_RTTS_IMPL_H_
