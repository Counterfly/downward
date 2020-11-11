#include "pure_rrw_fixed_probability.h"
// #include "../restart_strategy.h"

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

#define UNUSED(expr) do { (void)(expr); } while (0)
using namespace std;


namespace pure_rrw_fp {

	PureRRWFixedProb::PureRRWFixedProb(
		const options::Options &opts)
		: SearchEngine(opts),
		scaling_heuristic(opts.get<shared_ptr<Evaluator>>("scaling_heuristic")),
		scaling_factor(0),
		preferred_operator_heuristics(opts.get_list<shared_ptr<Evaluator>>("preferred")),
		prob(opts.get<double>("prob")),
		probability_preferred(opts.get<double>("pref_prob")),
		current_eval_context(state_registry.get_initial_state(), &statistics),
		plan(),
		last_num_expanded(-1),
      	rng(utils::parse_rng_from_options(opts)) {
		this->use_preferred = preferred_operator_heuristics.size() > 0;

		// struct timeval time;
		// gettimeofday(&time,NULL);
		// // microsecond has 1 000 000
		// // Assuming you did not need quite that accuracy
		// // Also do not assume the system clock has that accuracy.
		// unsigned int seed = (time.tv_sec * 1000) + (time.tv_usec / 1000);
		// srand(seed);

		utils::g_log << "----" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------------------------------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----" << endl;
		utils::g_log << "rng-random_seed: " << opts.get<int>("random_seed") << endl;
		utils::g_log << "Prob (as double) = " << prob << endl;
		utils::g_log << "Pref_Prob (as double) = " << probability_preferred << endl;
	}

	PureRRWFixedProb::~PureRRWFixedProb() {
	}

	void PureRRWFixedProb::reach_state(
		const GlobalState &parent, const OperatorID &op, const GlobalState &state) {
		UNUSED(parent);
		UNUSED(op);
		UNUSED(state);
	}

	void PureRRWFixedProb::save_plan_if_necessary() {
		if (found_solution()) {
			plan_manager.save_plan(plan, task_proxy);
		}
	}

	void PureRRWFixedProb::initialize() {
		utils::g_log << "Conducting enforced hill-climbing search, (real) bound = "
			<< bound << endl;

		// Re-initialize
		solution_found = false;
		plan.clear();
		d_counts.clear();

		SearchNode node = search_space.get_node(current_eval_context.get_state());
		node.open_initial();

		this->scaling_factor = current_eval_context.get_evaluator_value(this->scaling_heuristic.get());
		assert(this->scaling_factor > 0);
	}

	vector<OperatorID> PureRRWFixedProb::get_successors(
		EvaluationContext &eval_context) {
		vector<OperatorID> ops;
		successor_generator.generate_applicable_ops(eval_context.get_state(), ops);
		statistics.inc_expanded();
		statistics.inc_generated_ops(ops.size());
		// Randomize ops
		rng->shuffle(ops);
		return ops;
	}


	vector<OperatorID> PureRRWFixedProb::get_biased_successors(EvaluationContext &eval_context) {
		vector<OperatorID> ops;
		successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

		std::unordered_set<int> pref_op_ids;
		if (use_preferred) {
			for (shared_ptr<Evaluator> pref_heuristic : preferred_operator_heuristics) {
				const vector<OperatorID> &pref_ops1 = eval_context.get_preferred_operators(pref_heuristic.get());
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
				double r = (*rng)();
				// utils::g_log << "randomed...." << r << endl;
				if (r < probability_preferred) {
					// utils::g_log << "randoming among preferred" << endl;
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
		rng->shuffle(ops);
		return ops;
	}

	void PureRRWFixedProb::expand(EvaluationContext &eval_context, int d) {
		UNUSED(eval_context);
		UNUSED(d);
	}

	bool PureRRWFixedProb::check_goal_and_set_plan(const GlobalState &state) {
		if (task_properties::is_goal_state(task_proxy, state)) {
			utils::g_log << "Solution found!" << endl;
			set_plan(plan);
			return true;
		}
		return false;
	}

	void PureRRWFixedProb::search() {
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


	SearchStatus PureRRWFixedProb::step() {
		last_num_expanded = statistics.get_expanded();
		search_progress.check_progress(current_eval_context);

		if (check_goal_and_set_plan(current_eval_context.get_state())) {
			return SOLVED;
		}

		//expand(current_eval_context, 0);
		return ehc();
	}

	SearchStatus PureRRWFixedProb::ehc() {
		// NOT ACTUALLY EHC, just a pure RW
		// open list is empty..perform RWs or return no solution after enough trials
		EvaluationContext eval_context = current_eval_context;

		while (!check_goal_and_set_plan(eval_context.get_state()))
		{
			if (plan.size() >= this->scaling_factor) {
				/*
				* The behaviour for scaling is to go to depth 'scaling_factor' with probability 1, then for each
				* expansion thereafter, restart with probability 'prob'
				*/
				double random_value = (*rng)();	// Map values to probabilities
				if (random_value < prob) {
					// Restart!
					//utils::g_log << "restarting with length " << plan.size() << " and factor = " << this->scaling_factor << endl;
					eval_context = current_eval_context;
					plan.clear();
				}
			}

			if (timer->is_expired()) {
				// This will never happen, timer handled by system
				utils::g_log << "doing RW and calling timeout" << endl;
				return TIMEOUT;
			}
			//utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
			vector<OperatorID> ops = get_biased_successors(eval_context);
			if (ops.size() == 0) {
				utils::g_log << "Pruned all operators -- forcing early restart" << endl;
				//exit_with(EXIT_UNSOLVED_INCOMPLETE);
				eval_context = current_eval_context;
			}
			else {
				int random_op_id_index = (*rng)(ops.size());
				OperatorID random_op_id = ops.at(random_op_id_index);
				OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
				GlobalState state = state_registry.get_successor_state(eval_context.get_state(), random_op);

				//reach_state(eval_context.get_state(), *random_op, state);

				eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
				statistics.inc_evaluated_states();	// Evaluating random state
				statistics.inc_expanded();	// Expanding current state
				statistics.inc_generated();	// Only generating one (random) state
				// should inc_expanded_states() or inc_generated_states()?

				plan.push_back(random_op_id);
			}
		}

		current_eval_context = eval_context;
		return SOLVED;
		//utils::g_log << "No solution - FAILED" << endl;
		//return FAILED;
	}

	long PureRRWFixedProb::luby_sequence(long sequence_number) {
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

	void PureRRWFixedProb::print_statistics() const {
		statistics.print_detailed_statistics();
		utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;

		// utils::g_log << "For last execution: " << endl;
		// for (auto count : d_counts) {
		// 	int depth = count.first;
		// 	int phases = count.second.first;
		// 	assert(phases != 0);
		// 	int total_expansions = count.second.second;
		// 	utils::g_log << "EHC phases of depth " << depth << ": " << phases
		// 		<< " - Avg. Expansions: "
		// 		<< static_cast<double>(total_expansions) / phases << endl;
		// }
	}

	static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
		parser.document_synopsis("RRW with fixed probability restarts and optional heuristic scaling", "");

		parser.add_option<shared_ptr<Evaluator>>(
				"scaling_heuristic",
				"How to scale each RW before using the fixed probability restarts",
				"const(1)");
		// shared_ptr<Evaluator>>
		parser.add_list_option<shared_ptr<Evaluator>>(
				"preferred",
				"use preferred operators of these heuristics",
				"[]");
		parser.add_option<double>("prob", "fixed probability rate [0,1]", "0");
		parser.add_option<double>("pref_prob", "currently, not used. probability of selecting a preferred operator in local minima (to create distributions. 0 means zero probability of selecting a preferred operator. -1 means do not add any bias (treat pref and unpref the same). Valid input is [0,1])", "-1");
		utils::add_rng_options(parser);

		SearchEngine::add_options_to_parser(parser);
		Options opts = parser.parse();

		if (parser.dry_run())
			return nullptr;
		else
			return make_shared<PureRRWFixedProb>(opts);
	}

	static Plugin<SearchEngine> _plugin("pure_rrw_fixed_prob", _parse);
}