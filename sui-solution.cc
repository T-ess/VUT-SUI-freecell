#include "search-strategies.h"
#include <iostream>
#include <deque>
#include <set>
#include <map>
#include <algorithm>
#include "memusage.h"
using namespace std;

struct stateInfo {
	shared_ptr<SearchState> parent;
	shared_ptr<SearchAction> action;
	int depth;
};

struct stateInfoBFS {
	shared_ptr<SearchState> parent;
	shared_ptr<SearchAction> action;
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
// variables
	deque<shared_ptr<SearchState>> BFSopen;
	set<SearchState> BFSclosed;
	map<shared_ptr<SearchState>,stateInfoBFS> statesMap;
	stateInfoBFS info;

	// initial state
	shared_ptr<SearchState> init = make_shared<SearchState>(init_state);
	BFSopen.push_front(init);
	stateInfoBFS initial = {nullptr, nullptr};
	statesMap.emplace(init, initial);

	if (BFSopen.empty()) return {};
	shared_ptr<SearchState> actState;
	while (!BFSopen.empty() && !BFSopen.back()->isFinal()) {
		if ((getCurrentRSS() + 300000) >= mem_limit_) return {}; // memory check
		if (statesMap.count(BFSopen.back()) == 0) return {}; // current state is not in statesMap
		actState = BFSopen.back();
		if (BFSclosed.find(*actState) != BFSclosed.end()) { // current state already visited
			BFSopen.pop_back();
			continue;
		}
		BFSclosed.insert(*actState);
		BFSopen.pop_back();
		// expand current state and push new states
		vector<SearchAction> currAct = actState->actions();
		for (auto &gameMove : currAct) {
			SearchState newState = move(gameMove.execute(*actState));
			shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
			if(BFSclosed.find(*newStatePtr) != BFSclosed.end()) continue; // state found in CLOSED
			BFSopen.push_front(newStatePtr);
			info = {actState, make_shared<SearchAction>(gameMove)};
			statesMap.emplace(newStatePtr, info);
		}
	}
	// retrace steps and get actions
	vector<SearchAction> finalActions;
	if (!BFSopen.empty() && BFSopen.back()->isFinal()) {
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = BFSopen.back();
		while (statesMap.at(state).action != nullptr) {
			if (statesMap.count(state) == 0) return {};
			// insert the next action to the beginning of the vector
			finalActions.push_back(*(statesMap.at(state).action));
			rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
			state = statesMap.at(state).parent; // get the next state
		}
		return finalActions;
	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<shared_ptr<SearchState>> DFSopen;
	set<SearchState> DFSclosed;
	map<shared_ptr<SearchState>,stateInfo> statesMap;
	stateInfo info;

	// initial state
	shared_ptr<SearchState> init = make_shared<SearchState>(init_state);
	DFSopen.push_back(init);
	stateInfo initial = {nullptr, nullptr, 0};
	statesMap.emplace(init, initial);

	if (DFSopen.empty()) return {};
	shared_ptr<SearchState> actState;
	while (!DFSopen.empty() && !DFSopen.back()->isFinal()) {
		if ((getCurrentRSS() + 300000) >= mem_limit_) return {}; // memory check
		if (statesMap.count(DFSopen.back()) == 0) return {}; // current state is not in statesMap
		int actDepth = statesMap.at(DFSopen.back()).depth;
		if (actDepth < depth_limit_) { // depth limit check
			actState = DFSopen.back();
			if (DFSclosed.find(*actState) != DFSclosed.end()) { // current state already visited
				DFSopen.pop_back();
				continue;
			}
			DFSclosed.insert(*actState);
			DFSopen.pop_back();
			// expand current state and push new states
			vector<SearchAction> currAct = actState->actions();
			int newStateDepth = statesMap.at(actState).depth + 1;
			for (auto &gameMove : currAct) {
				SearchState newState = move(gameMove.execute(*actState));
				shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
				if(DFSclosed.find(*newStatePtr) != DFSclosed.end()) continue; // state found in CLOSED
				DFSopen.push_back(newStatePtr);
				info = {actState, make_shared<SearchAction>(gameMove), newStateDepth};
				statesMap.emplace(newStatePtr, info);
			}
		} else { // the depth limit has been reached
			DFSopen.pop_back();
		}
	}
	// retrace steps and get actions
	vector<SearchAction> finalActions;
	if (!DFSopen.empty() && DFSopen.back()->isFinal()) {
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = DFSopen.back();
		while (statesMap.at(state).action != nullptr) {
			if (statesMap.count(state) == 0) return {};
			// insert the next action to the beginning of the vector
			finalActions.push_back(*(statesMap.at(state).action));
			rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
			state = statesMap.at(state).parent; // get the next state
		}
		return finalActions;
	}
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
