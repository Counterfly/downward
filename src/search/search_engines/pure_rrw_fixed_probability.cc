#include "pure_rrw_fixed_probability.h"
// #include "../restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../algorithms/ordered_set.h"
#include "../evaluators/const_evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../operator_id.h"
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
		preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
		restart_probability(opts.get<double>("prob")),
		probability_preferred(opts.get<double>("pref_prob")),
    	num_restarts(0),
		current_eval_context(state_registry.get_initial_state(), &statistics),
		last_num_expanded(-1),
      	rng(utils::parse_rng_from_options(opts)) {
		this->use_preferred = preferred_operator_evaluators.size() > 0;

		utils::g_log << "----" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------------------------------" << endl;
		utils::g_log << "----------------" << endl;
		utils::g_log << "--------" << endl;
		utils::g_log << "----" << endl;
		utils::g_log << "rng-random_seed: " << opts.get<int>("random_seed") << endl;
		utils::g_log << "Pref_Prob (as double) = " << probability_preferred << endl;
    	utils::g_log << "Use Preferred =  " << use_preferred << endl;
    	utils::g_log << "Restart_Prob (as double) = " << restart_probability << endl;
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
    	num_restarts = 0;

		SearchNode node = search_space.get_node(current_eval_context.get_state());
		node.open_initial();

		this->scaling_factor = current_eval_context.get_evaluator_value(this->scaling_heuristic.get());
		assert(this->scaling_factor > 0);
	}

void PureRRWFixedProb::get_biased_successors(EvaluationContext &eval_context, ordered_set::OrderedSet<OperatorID> &ops) const {
	unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> pref_ops;
    unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> non_pref_ops;
    // if there are no preferred_operator_evaluators 
    // then pref_ops will be empty, and non_pref_ops should contain ops
    successor_generator.get_partitioned_operators(
        eval_context,
        preferred_operator_evaluators,
        pref_ops,
        non_pref_ops
    );
	// utils::g_log << "Pref Ops.size v Non-Pref Ops.size: " << pref_ops.size() << " v " << non_pref_ops.size() << endl;

    // favouring to update stats after random number is generated (and now outside of this function bc of an old const battle)
    // b/c, in theory, we wouldn't need to generate the unselected operator set
    // statistics.inc_expanded();
    // statistics.inc_generated_ops(pref_ops.size() + non_pref_ops.size());
    if (use_preferred && probability_preferred > 0 && pref_ops.size() > 0) {
        // Both operator types exist, randomly choose between the two sets
        double r = (*rng)();
        // utils::g_log << "randomed...." << r << endl;
        if (r < probability_preferred) {
            // utils::g_log << "randoming among preferred" << endl;
            for (OperatorID op : pref_ops) {
                ops.insert(op);
            }
        }
        else {
            // utils::g_log << "randoming among non_pref" << endl;
            for (OperatorID op : non_pref_ops) {
                ops.insert(op);
            }
        }
    }
    else {
        for (OperatorID op : pref_ops) {
            ops.insert(op);
        }
        for (OperatorID op : non_pref_ops) {
            ops.insert(op);
        }
        // utils::g_log << "randoming among both pref and non-pref ops with ops.size = " << ops.size() << endl;
    }

    // Randomize ops
    // ops.shuffle(*rng); -- this didn't work
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
				* expansion thereafter, restart with probability 'restart_probability'
				*/
				double random_value = (*rng)();	// Map values to probabilities
				if (random_value < restart_probability) {
					// Restart!
					//utils::g_log << "restarting with length " << plan.size() << " and factor = " << this->scaling_factor << endl;
					eval_context = current_eval_context;
					plan.clear();
            		++num_restarts;
				}
			}

			if (timer->is_expired()) {
				// This will never happen, timer handled by system
				utils::g_log << "doing RW and calling timeout" << endl;
				return TIMEOUT;
			}
			//utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
			ordered_set::OrderedSet<OperatorID> ops;
			get_biased_successors(eval_context, ops);
			if (ops.size() == 0) {
				utils::g_log << "Pruned all operators -- forcing early restart" << endl;
				//exit_with(EXIT_UNSOLVED_INCOMPLETE);
				eval_context = current_eval_context;
			}
			else {
				// randomly select op
				int random_op_id_index = (*rng)(ops.size());
				OperatorID random_op_id = ops[random_op_id_index];
				OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
				GlobalState state = state_registry.get_successor_state(eval_context.get_state(), random_op);

				//reach_state(eval_context.get_state(), *random_op, state);

				eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
				// statistics.inc_evaluated_states();	// Evaluating random state
				statistics.inc_expanded();	// Expanding current state
				statistics.inc_generated();	// Only generating one (random) state
				statistics.inc_generated_ops(ops.size());
				// should inc_expanded_states() or inc_generated_states()?

				plan.push_back(random_op_id);
			}
		}

		current_eval_context = eval_context;
		return SOLVED;
		//utils::g_log << "No solution - FAILED" << endl;
		//return FAILED;
	}

	void PureRRWFixedProb::print_statistics() const {
		statistics.print_detailed_statistics();
		utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;
		utils::g_log << "Number of Restarts: " << num_restarts << "." << endl;

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