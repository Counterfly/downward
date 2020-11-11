#include "pure_rw.h"

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

namespace pure_rw {

PureRW::PureRW(
    const Options &opts)
    : SearchEngine(opts),
      current_eval_context(state_registry.get_initial_state(), &statistics),
      last_num_expanded(-1),
      rng(utils::parse_rng_from_options(opts)) {

	utils::g_log << "----" << endl;
	utils::g_log << "--------" << endl;
	utils::g_log << "----------------" << endl;
	utils::g_log << "--------------------------------" << endl;
	utils::g_log << "----------------" << endl;
	utils::g_log << "--------" << endl;
	utils::g_log << "----" << endl;
	utils::g_log << "rng-random_seed: " << opts.get<int>("random_seed") << endl;
}

PureRW::~PureRW() {
}

void PureRW::reach_state(
    const GlobalState &parent, const OperatorID &op, const GlobalState &state) {
	UNUSED(parent);
	UNUSED(op);
	UNUSED(state);
}

void PureRW::save_plan_if_necessary() {
	if (found_solution()) {
		plan_manager.save_plan(plan, task_proxy);
	}
}

void PureRW::initialize() {
    utils::g_log << "Conducting enforced hill-climbing search, (real) bound = "
         << bound << endl;

    // Re-initialize
    solution_found = false;
	plan.clear();
    d_counts.clear();

    SearchNode node = search_space.get_node(current_eval_context.get_state());
    node.open_initial();
    utils::g_log << "finitial initialize, current_Eval_context.id = " << current_eval_context.get_state().get_id() << endl;
}

vector<OperatorID> PureRW::get_successors(
    EvaluationContext &eval_context) {
    vector<OperatorID> ops;
	successor_generator.generate_applicable_ops(eval_context.get_state(), ops);
    statistics.inc_expanded();
    statistics.inc_generated_ops(ops.size());
    // Randomize ops
	rng->shuffle(ops);
    return ops;
}


vector<OperatorID> PureRW::get_biased_successors(
    EvaluationContext &eval_context) {
	vector<OperatorID> ops;
	// g_successor_generator->generate_applicable_ops(eval_context.get_state(), ops);
	successor_generator.generate_applicable_ops(eval_context.get_state(), ops);

    statistics.inc_expanded();
    statistics.inc_generated_ops(ops.size());

    // Randomize ops
    rng->shuffle(ops);
    return ops;
}

void PureRW::expand(EvaluationContext &eval_context, int d) {
	UNUSED(eval_context);
	UNUSED(d);
}

bool PureRW::check_goal_and_set_plan(const GlobalState &state) {
    if (task_properties::is_goal_state(task_proxy, state)) {
        utils::g_log << "Solution found!" << endl;
        set_plan(plan);
        return true;
    }
    return false;
}

void PureRW::search() {
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


	current_eval_context = EvaluationContext(state_registry.get_initial_state(), &statistics);
}


SearchStatus PureRW::step() {
    last_num_expanded = statistics.get_expanded();
    search_progress.check_progress(current_eval_context);

    if (check_goal_and_set_plan(current_eval_context.get_state())) {
    	utils::g_log << "step, returning SOLVED" << endl;
        return SOLVED;
    }

    //expand(current_eval_context, 0);
    return ehc();
}

SearchStatus PureRW::ehc() {
	// NOT ACTUALLY EHC, just a pure RW
    // open list is empty..perform RWs or return no solution after enough trials
    EvaluationContext eval_context = current_eval_context;

    eval_context = current_eval_context;
	while (!check_goal_and_set_plan(eval_context.get_state()))
	{
		if (timer->is_expired()) {
			// This will never happen, timer handled by system
			utils::g_log << "doing RW and calling timeout" << endl;
			// utils::exit_with(utils::ExitCode::SEARCH_OUT_OF_TIME);
			return TIMEOUT;
		}
		//utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
		vector<OperatorID> ops = get_biased_successors(eval_context);
		// vector<OperatorID> ops;
		if (ops.size() == 0) {
			utils::g_log << "Pruned all operators -- exiting and marking as unsolvable" << endl;
			return FAILED;
			//eval_context = current_eval_context;
		}
		else {
			int random_op_id = (*rng)(ops.size());
			// utils::g_log << "Randomly selected operator with index = " << random_op_id << endl;
			OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
			GlobalState state = state_registry.get_successor_state(eval_context.get_state(), random_op);

			//reach_state(eval_context.get_state(), *random_op, state);

			eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
			statistics.inc_evaluated_states();	// Evaluating random state
			statistics.inc_expanded();	// Expanding current state
			statistics.inc_generated();	// Only generating one (random) state
			// should inc_expanded_states() or inc_generated_states()?

			plan.push_back(ops[random_op_id]);
			utils::g_log << "ops size,id = " << ops.size() << "," << random_op_id << "  new state: " << eval_context.get_state().get_id() << "plan length=" << plan.size() << endl;
		}
	}

    //for (std::vector<const GlobalState*>::iterator it = walk.begin(); it != walk.end(); ++it)
    //GlobalState parent_state = current_eval_context.get_state();
    //for (unsigned int i = 0; i < walk.size(); ++i)
    //for (GlobalState state : walk)
    //{
    	//GlobalState state = walk.at(i);
    	//utils::g_log << state.get_id() << "-->";
    	//const OperatorID* random_op = actions.at(i);
    	//const GobalState* state = *it;
    	//SearchNode search_node = search_space.get_node(state);
    	//search_node.update_parent(search_space.get_node(parent_state), random_op);
    	//parent_state = state;
    //}

    current_eval_context = eval_context;
    return SOLVED;
    //utils::g_log << "No solution - FAILED" << endl;
    //return FAILED;
}

long PureRW::luby_sequence(long sequence_number) {
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

void PureRW::print_statistics() const {
	statistics.print_detailed_statistics();
	utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;

	// currently not used
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
		parser.document_synopsis("Single RW, no restarts, uniform probability", "");
		// parser.add_option<int>("pref_prob", "currently, not used. probability of selecting a preferred operator in local minima (to create distributions. 0 means zero probability of selecting a preferred operator. -1 means do not add any bias (treat pref and unpref the same). Valid input is [0,100])", "-1");

		utils::add_rng_options(parser);
		SearchEngine::add_options_to_parser(parser);
		Options opts = parser.parse();

		if (parser.dry_run())
			return nullptr;
		else
			return make_shared<PureRW>(opts);
	}

	static Plugin<SearchEngine> _plugin("pure_rw", _parse);
}
