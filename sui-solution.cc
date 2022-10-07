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
	while (!BFSopen.empty() && !BFSopen.back().get()->isFinal()) {
		//printf("BFS: MemLimit %f%%    %lu / %lu\n", (double) getCurrentRSS() / mem_limit_ * 100, getCurrentRSS(), mem_limit_);
		if (statesMap.find(BFSopen.back()) == statesMap.end()) {
			// invalid
			return {};
		}
		shared_ptr<SearchState> actState = BFSopen.back();
		BFSclosed.insert(actState);
		BFSopen.pop_back();
		// expand and push new states
		vector<SearchAction> currAct = actState.get()->actions();
		if (statesMap.find(actState) == statesMap.end()) {
			// invalid
			return {};
		}
		for (auto &move : currAct) {
			SearchState newState = move.execute(*(actState.get()));
			// check if newState should be in stack (is not in open or closed)
			shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(newState);
			auto itr = BFSclosed.find(newStatePtr);
			if(itr != BFSclosed.end()) {
				// state was found in closed
				continue;
			}
			// auto itr2 = find(BFSopen.begin(), BFSopen.end(), newStatePtr);
			// if(itr2 != BFSopen.end()) {
			// 	// state was found in open
			// 	continue;
			// }

			bool openFound = false;
			for (auto &stateCheck : BFSopen) {
				if (operator==(*(stateCheck.get()), newState)) {
					openFound = true; // TODO fix this - padne to sem hned napoprve
					break;
				}
			}

			if (openFound) {
				continue;
			}
			// push the new state
			BFSopen.push_front(newStatePtr);
			// save the parent and action of new state
			
			info = {actState, make_shared<SearchAction>(move)};
			statesMap.emplace(newStatePtr, info);
		}
	}

	if (BFSopen.empty()) {
		return {};
	} else if (BFSopen.back().get()->isFinal()) {
		vector<SearchAction> finalActions;
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = BFSopen.back();

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
			//state found in closed
			// auto actInClosed = false;
			// for (auto &stateCheck : DFSclosed) {
			// 	if (operator==(stateCheck, *(actState.get()))) {			
			// 		actInClosed = true;
			// 		break;
			// 	}
			// }

			// if (actInClosed){
			// 	actInClosed = false;
			// 	DFSopen.pop_back();
			// 	continue;
			// }

			//DFSclosed.insert(*(actState.get()));
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
				// auto itr = DFSclosed.find(newState);
				// if(itr != DFSclosed.end()) {
				// 	// state was found in closed
				// 	continue;
				// }
				// auto itr2 = find(DFSopen.begin(), DFSopen.end(), newStatePtr);
				// if(itr2 != DFSopen.end()) {
				// 	// state was found in open
				// 	continue;
				// }

				bool openFound = false;
				for (auto &stateCheck : DFSopen) {
					if (operator==(*(stateCheck.get()), newState)) {
						openFound = true; // TODO fix this - padne to sem hned napoprve
						break;
					}
				}
				if (openFound) {
					openFound = false;
					continue;
				}


				if (statesMap[actState].parent != nullptr && operator==(*(statesMap[actState].parent.get()), newState)){
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

	vector<SearchAction> finalActions;
	if (DFSopen.empty()) {
		return {};
	} else if (DFSopen.back().get()->isFinal()) {
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = DFSopen.back();

		while (statesMap[state].action != nullptr) {
			if (statesMap.find(state) == statesMap.end()) {
				// invalid
				return {};
			} else {
				// insert the next action to the beginning of the vector
				finalActions.push_back(*(statesMap[state].action.get()));
				rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
				state = statesMap[state].parent;
			}
		}
	}
	return finalActions;
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
