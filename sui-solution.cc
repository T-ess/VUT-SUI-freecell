#include "search-strategies.h"
#include <iostream>
#include <deque>
#include <set>
#include <map>
#include <algorithm>
#include <memory>
using namespace std;

struct stateInfo {
	shared_ptr<SearchState> parent;
	shared_ptr<SearchAction> action;
	int depth;
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<shared_ptr<SearchState>> DFSopen;
	set<shared_ptr<SearchState>> DFSclosed;
	map<shared_ptr<SearchState>,stateInfo> statesMap;
	stateInfo info;

	// initial state initialization
	shared_ptr<SearchState> init = make_shared<SearchState>(init_state);
	DFSopen.push_back(init);
	stateInfo initial = {nullptr, nullptr, 0};
	statesMap.emplace(init, initial);

	if (DFSopen.empty()) return {};
	while (!DFSopen.empty() && !DFSopen.back().get()->isFinal()) {
		//TODO depth check
		int actDepth;
			if (statesMap.find(DFSopen.back()) == statesMap.end()) {
			// invalid
			return {};
		} else {
			actDepth = statesMap[DFSopen.back()].depth;
		}

		if (actDepth < depth_limit_) {
			shared_ptr<SearchState> actState = DFSopen.back();
			DFSclosed.insert(actState);
			DFSopen.pop_back();
			// expand and push new states
			vector<SearchAction> currAct = actState.get()->actions();
			int newStateDepth;
			if (statesMap.find(actState) == statesMap.end()) {
				// invalid
				return {};
			} else {
				newStateDepth = statesMap[actState].depth + 1;
			}
			for (auto &move : currAct) {
				SearchState newState = move.execute(*(actState.get()));
				// check if newState should be in stack (is not in open or closed)
				shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
				auto itr = DFSclosed.find(newStatePtr);
				if(itr != DFSclosed.end()) {
					// state was found in closed
					continue;
				}
				auto itr2 = find(DFSopen.begin(), DFSopen.end(), newStatePtr);
				if(itr2 != DFSopen.end()) {
					// state was found in open
					continue;
				}

				// push the new state
				DFSopen.push_back(newStatePtr);
				// save the parent and action of new state
				
				info = {actState, make_shared<SearchAction>(move), newStateDepth};
				statesMap.emplace(newStatePtr, info);
			}
		} else {
			DFSopen.pop_back();
		}
	}

	if (DFSopen.empty()) {
		return {};
	} else if (DFSopen.back().get()->isFinal()) {
		vector<SearchAction> finalActions;
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = DFSopen.back();

		while (true) {
			if (statesMap.find(state) == statesMap.end()) {
				// invalid
				return {};
			} else {
				if (statesMap[state].action == nullptr) {
					// the initial state
					return finalActions;
				} else {
					// insert the next action to the beginning of the vector
					finalActions.push_back(*(statesMap[state].action.get()));
					rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
					state = statesMap[state].parent;
				}
			}
		}

	}
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
