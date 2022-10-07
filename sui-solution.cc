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

bool operator==(const SearchState &a, const SearchState &b) {
    return a.state_ == b.state_;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<shared_ptr<SearchState>> BFSopen;
	set<shared_ptr<SearchState>> BFSclosed;
	map<shared_ptr<SearchState>,stateInfoBFS> statesMap;
	stateInfoBFS info;

	// initial state initialization
	shared_ptr<SearchState> init = make_shared<SearchState>(init_state);
	BFSopen.push_front(init);
	stateInfoBFS initial = {nullptr, nullptr};
	statesMap.emplace(init, initial);

	if (BFSopen.empty()) return {};
	while (!BFSopen.empty() && !BFSopen.back()->isFinal()) {
		printf("BFS: MemLimit %f%%    %lu / %lu\n", (double) getCurrentRSS() / mem_limit_ * 100, getCurrentRSS(), mem_limit_);
		if (statesMap.find(BFSopen.back()) == statesMap.end()) {
			// invalid
			return {};
		}
		shared_ptr<SearchState> actState = BFSopen.back();
		BFSclosed.insert(actState);
		BFSopen.pop_back();
		// expand and push new states
		vector<SearchAction> currAct = actState->actions();
		if (statesMap.find(actState) == statesMap.end()) {
			// invalid
			return {};
		}
		for (auto &gameMove : currAct) {
			SearchState newState = gameMove.execute(*actState);
			// check if newState should be in stack (is not in open or closed)
			shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
			auto itr = BFSclosed.find(newStatePtr);
			if(itr != BFSclosed.end()) {
				// state was found in closed
				continue;
			}
			auto itr2 = find(BFSopen.begin(), BFSopen.end(), newStatePtr);
			if(itr2 != BFSopen.end()) {
				// state was found in open
				continue;
			}

			// push the new state
			BFSopen.push_front(newStatePtr);
			// save the parent and action of new state
			
			info = {actState, make_shared<SearchAction>(gameMove)};
			statesMap.emplace(newStatePtr, info);
		}
	}

	if (BFSopen.empty()) {
		return {};
	} else if (BFSopen.back()->isFinal()) {
		vector<SearchAction> finalActions;
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = BFSopen.back();

		while (true) {
			if (statesMap.find(state) == statesMap.end()) {
				// invalid
				return {};
			} else {
				if (statesMap.at(state).action == nullptr) {
					// the initial state
					return finalActions;
				} else {
					// insert the next action to the beginning of the vector
					finalActions.push_back(*(statesMap.at(state).action));
					rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
					state = statesMap.at(state).parent;
				}
			}
		}

	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<shared_ptr<SearchState>> DFSopen;
	set<SearchState> DFSclosed;
	map<shared_ptr<SearchState>,stateInfo> statesMap;
	stateInfo info;

	// initial state initialization
	shared_ptr<SearchState> init = make_shared<SearchState>(init_state);
	DFSopen.push_back(init);
	stateInfo initial = {nullptr, nullptr, 0};
	statesMap.emplace(init, initial);

	if (DFSopen.empty()) return {};
	shared_ptr<SearchState> actState;
	while (!DFSopen.empty() && !DFSopen.back()->isFinal()) {
		if ((getCurrentRSS() + 300000) >= mem_limit_) return {};
		if (statesMap.count(DFSopen.back()) == 0) return {};
		int actDepth = statesMap.at(DFSopen.back()).depth;
		if (actDepth < depth_limit_) {
			actState = DFSopen.back();
			if (DFSclosed.find(*actState) != DFSclosed.end()) {
				DFSopen.pop_back();
				continue;
			}
			DFSclosed.insert(*actState);
			DFSopen.pop_back();
			// expand and push new states
			vector<SearchAction> currAct = actState->actions();
			int newStateDepth = 0;
			if (statesMap.count(actState) == 0) {
				// invalid
				cout << "invalid" << endl;
				return {};
			} else {
				newStateDepth = statesMap.at(actState).depth + 1;
			}
			for (auto &gameMove : currAct) {
				SearchState newState = move(gameMove.execute(*actState));
				// check if newState should be in stack (is not in open or closed)
				shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
				if(DFSclosed.find(*newStatePtr) != DFSclosed.end()) continue; // state found in CLOSED

				DFSopen.push_back(newStatePtr);
				info = {actState, make_shared<SearchAction>(gameMove), newStateDepth};
				statesMap.emplace(newStatePtr, info);
			}
		} else {
			DFSopen.pop_back();
		}
	}

	vector<SearchAction> finalActions;
	if (DFSopen.empty()) {
		return {};
	} else if (DFSopen.back()->isFinal()) {
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = DFSopen.back();

		while (statesMap.at(state).action != nullptr) {
			if (statesMap.find(state) == statesMap.end()) {
				// invalid
				return {};
			} else {
				// insert the next action to the beginning of the vector
				finalActions.push_back(*(statesMap.at(state).action));
				rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
				state = statesMap.at(state).parent;
			}
		}
	}
	statesMap.clear();
	DFSclosed.clear();
	DFSopen.clear();
	return finalActions;
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
