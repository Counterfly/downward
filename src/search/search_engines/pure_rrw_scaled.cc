#include "pure_rrw_scaled.h"

#include "../restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../evaluators/const_evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../operator_id.h"
#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include <algorithm>  // for random_shuffle
#include <cstdint>
#include "stdio.h"	// for NULL
#include "stdlib.h" // for rand() and srand
#include "time.h"	// for time

#include <sys/time.h>
#define UNUSED(expr) do { (void)(expr); } while (0)
using namespace std;

namespace pure_rrw_scaled {

	PureRRWScaled::PureRRWScaled(
		const Options &opts)
		: SearchEngine(opts),
		initial_heuristic(opts.get<shared_ptr<Evaluator>>("h")),
		// initial_heuristic_value(-1),
		preferred_operator_heuristics(opts.get_list<shared_ptr<Evaluator>>("preferred")),
		restart_strategy(opts.get<shared_ptr<RestartStrategy>>("restart")),
		probability_preferred(opts.get<double>("pref_prob")),
		current_eval_context(state_registry.get_initial_state(), &statistics),
		plan(),
		last_num_expanded(-1) {

		this->use_preferred = preferred_operator_heuristics.size() > 0;


		// else prefusage == DO_NOTHING
		struct timeval time;
		gettimeofday(&time,NULL);

		// microsecond has 1 000 000
		// Assuming you did not need quite that accuracy
		// Also do not assume the system clock has that accuracy.
		unsigned int seed = (time.tv_sec * 1000) + (time.tv_usec / 1000);
		srand(seed);

		utils::g_log << "----" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------------------------------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----" << endl;
		utils::g_log << "srand-SEED: " << seed << endl;
		utils::g_log << "Pref_Prob (as double) = " << probability_preferred << endl;
		utils::g_log << "Use Preferred =  " << use_preferred << endl;
	}

	PureRRWScaled::~PureRRWScaled() {
	}

	void PureRRWScaled::reach_state(
		const GlobalState &parent, const OperatorID &op, const GlobalState &state) {
		UNUSED(parent);
		UNUSED(op);
		UNUSED(state);
	}

	void PureRRWScaled::save_plan_if_necessary() {
		if (found_solution()) {
			plan_manager.save_plan(plan, task_proxy);
		}
	}

	void PureRRWScaled::initialize() {
		utils::g_log << "Conducting enforced hill-climbing search, (real) bound = "
			<< bound << endl;

		// Re-initialize
		solution_found = false;
		plan.clear();
		d_counts.clear();

		SearchNode node = search_space.get_node(current_eval_context.get_state());
		node.open_initial();


		initial_heuristic_value = current_eval_context.get_evaluator_value(this->initial_heuristic.get());
		assert(initial_heuristic_value > 0);
	}

	vector<OperatorID> PureRRWScaled::get_successors(
		EvaluationContext &eval_context) {
		vector<OperatorID> ops;
		successor_generator.generate_applicable_ops(eval_context.get_state(), ops);
		statistics.inc_expanded();
		statistics.inc_generated_ops(ops.size());
		// Randomize ops
		std::random_shuffle(ops.begin(), ops.end());
		// ops.shuffle(*rng);
		return ops;
	}


	vector<OperatorID> PureRRWScaled::get_biased_successors(
		EvaluationContext &eval_context) {
		vector<OperatorID> ops;
			successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

			std::unordered_set<int> pref_op_ids;
			if (use_preferred) {
				for (shared_ptr<Evaluator> pref_heuristic : preferred_operator_heuristics) {
					const vector<OperatorID> &pref_ops1 =
						eval_context.get_preferred_operators(pref_heuristic.get());
					//utils::g_log << "pref heur = " << pref_heuristic->get_description() << " num pref ops = " << pref_ops1.size() << endl;
					for (OperatorID op : pref_ops1) {
						pref_op_ids.insert(op.get_index());
					}
				}
			}

			std::unordered_set<int> non_pref_op_ids;
			for (OperatorID op : ops) {
				int index = op.get_index();
				if (pref_op_ids.find(index) == pref_op_ids.end()) {
					non_pref_op_ids.insert(index);
				}
			}

			statistics.inc_expanded();
			statistics.inc_generated_ops(ops.size());

			if (probability_preferred != -1) {
				ops.clear();

				// Before doing randomization, see if one list is empty which forces deterministic choice
				if (pref_op_ids.size() == 0) {
					//utils::g_log << "Pref Operators is empty" << endl;
					for (int op_id : non_pref_op_ids) {
						ops.push_back(OperatorID(op_id));
					}
				}
				else if (non_pref_op_ids.size() == 0) {
					//utils::g_log << "NonPref Operators is empty" << endl;
					for (int op_id : pref_op_ids) {
						ops.push_back(OperatorID(op_id));
					}
				}
				else {
					// Both operator types exist, randomly choose between the two sets
					//int r = (rand() % 100);
					double r = this->get_probability();
					//utils::g_log << "randomed...." << r << endl;
					if (r < probability_preferred) {
						//utils::g_log << "randoming among preferred" << endl;
						for (int op_id : pref_op_ids) {
							ops.push_back(OperatorID(op_id));
						}
					}
					else {
						//utils::g_log << "randoming among non_pref" << endl;
						for (int op_id : non_pref_op_ids) {
							ops.push_back(OperatorID(op_id));
						}
					}
				}
				/*
				// Bias operators for appropriate distribution
				ops.clear();

				for (int i = 0; i < this->instances_non_preferred; ++i){
					for (const GlobalOperator * op : non_pref_ops) {
						ops.push_back(op);
					}
				}
				utils::g_log << "instances-non-pref = " << this->instances_non_preferred << ", num non-pref ops = " << non_pref_ops.size() << ", ops size = " << ops.size() << endl;

				for (int i = 0; i < this->instances_preferred; ++i){
					for (const GlobalOperator * op : pref_ops) {
						ops.push_back(op);
					}
				}
				*/
				//utils::g_log << "num pref ops = " << pref_ops.size() << ", new ops size = " << ops.size() << endl;
			}

			// Randomize ops
			std::random_shuffle(ops.begin(), ops.end());
			// ops.shuffle(*rng);
			return ops;
	}

	void PureRRWScaled::expand(EvaluationContext &eval_context, int d) {
		UNUSED(eval_context);
		UNUSED(d);
	}

	bool PureRRWScaled::check_goal_and_set_plan(const GlobalState &state) {
		if (task_properties::is_goal_state(task_proxy, state)) {
			utils::g_log << "Solution found!" << endl;
			set_plan(plan);
			return true;
		}
		return false;
	}

	void PureRRWScaled::search() {
		initialize();
		status = IN_PROGRESS;
		timer = new utils::CountdownTimer(this->max_time);	// Use as private instance variable so don't have to pass it through function
		while (status == IN_PROGRESS) {
			status = step();
			if (timer->is_expired()) {
				// Timer is done through system (python scripts check call.py, limits.py)
				// Timer will always be inf....stupid
				utils::g_log << "Time limit reached. Abort search." << endl;
				status = TIMEOUT;
				break;
			}
		}

		// utils::g_log << "Actual search time: " << *timer << " [t=" << g_timer << "]" << endl;
		utils::g_log << "Actual search time: " << timer->get_elapsed_time() << endl;
		delete timer;
	}


	SearchStatus PureRRWScaled::step() {
		last_num_expanded = statistics.get_expanded();
		search_progress.check_progress(current_eval_context);

		if (check_goal_and_set_plan(current_eval_context.get_state())) {
			return SOLVED;
		}

		if (initial_heuristic_value < 1) {
			exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}
		//expand(current_eval_context, 0);
		return ehc();
	}

	SearchStatus PureRRWScaled::ehc() {
		// NOT ACTUALLY EHC, just a pure RW
		// open list is empty..perform RWs or return no solution after enough trials
		EvaluationContext eval_context = current_eval_context;

		uint64_t MAX_TIMESTEP = 1;
		MAX_TIMESTEP <<= 63;

		while (!check_goal_and_set_plan(eval_context.get_state()))
		{

			uint64_t restart_length = restart_strategy->next_sequence_value();
			// utils::g_log << "restart length = " << restart_length << ", " << endl;
			if (restart_length < (MAX_TIMESTEP / initial_heuristic_value)) {
				restart_length = restart_length * initial_heuristic_value;
			}
			else {
				restart_length = MAX_TIMESTEP;
			}

			//utils::g_log << "new restart length = " << restart_length << endl;
			uint64_t timestep = 0;
			eval_context = current_eval_context;
			plan.clear();
			while (timestep < restart_length && !check_goal_and_set_plan(eval_context.get_state()))
			{
				++timestep;
				if (timer->is_expired()) {
					// This will never happen, timer handled by system
					utils::g_log << "timeout reached during a RW" << endl;
					return TIMEOUT;
				}
				//utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
				vector<OperatorID> ops = get_biased_successors(eval_context);
				if (ops.size() == 0) {
					utils::g_log << "Pruned all operators -- resetting current state to EHC root" << endl;
					eval_context = current_eval_context;
					plan.clear(); // TODO: this wasn't in old
				}
				else {
					OperatorID random_op_id = ops.at(rand() % ops.size());
					OperatorProxy random_op = task_proxy.get_operators()[random_op_id];

					GlobalState state = state_registry.get_successor_state(eval_context.get_state(), random_op);

					//reach_state(eval_context.get_state(), random_op, state);

					eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
					statistics.inc_evaluated_states();	// Evaluating random state
					statistics.inc_expanded();	// Expanding current state
					statistics.inc_generated();	// Only generating one (random) state
					// should inc_expanded_states() or inc_generated_states()?

					plan.push_back(random_op_id);
				}
			}
		}

		current_eval_context = eval_context;
		return SOLVED;
		//utils::g_log << "No solution - FAILED" << endl;
		//return FAILED;
	}

	long PureRRWScaled::luby_sequence(long sequence_number) {
		long focus = 2L;
		while (sequence_number > (focus - 1)) {
			focus = focus << 1;
		}

		if (sequence_number == (focus - 1)) {
			return focus >> 1;
		}
		else {
			return luby_sequence(sequence_number - (focus >> 1) + 1);
		}
	}

	void PureRRWScaled::print_statistics() const {
		statistics.print_detailed_statistics();
		utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;

		// utils::g_log << "For last execution: " << endl;
		for (auto count : d_counts) {
			int depth = count.first;
			int phases = count.second.first;
			assert(phases != 0);
			int total_expansions = count.second.second;
			utils::g_log << "EHC phases of depth " << depth << ": " << phases
				<< " - Avg. Expansions: "
				<< static_cast<double>(total_expansions) / phases << endl;
		}
	}

	static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
		parser.document_synopsis("Single RW, restarting RWs with sequence-based restarts. If h is supplied then its value from the initial state is multiplied to the restarting sequence.  Uniform probability", "");
		parser.add_option<shared_ptr<Evaluator>>("h", "heuristic. default is to no scale (scale by 1)", "const(1)");
		parser.add_list_option<shared_ptr<Evaluator>>(
				"preferred",
				"use preferred operators of these heuristics",
				"[]");
		parser.add_option<shared_ptr<RestartStrategy>>("restart", "restart strategy (luby [default] or exponential)", "luby");
		parser.add_option<double>("pref_prob", "currently, not used. probability of selecting a preferred operator in local minima (to create distributions. 0 means zero probability of selecting a preferred operator. -1 means do not add any bias (treat pref and unpref the same). Valid input is [0,100])", "-1");

		SearchEngine::add_options_to_parser(parser);
		Options opts = parser.parse();

		if (parser.dry_run())
			return nullptr;
		else
			return make_shared<PureRRWScaled>(opts);
	}

	static Plugin<SearchEngine> _plugin("pure_rrw_scaled", _parse);

}