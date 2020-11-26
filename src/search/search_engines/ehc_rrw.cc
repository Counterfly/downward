#include "ehc_rrw.h"

#include "../restart_strategy.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../algorithms/ordered_set.h"
#include "../evaluators/const_evaluator.h"
#include "../evaluators/pref_evaluator.h"
#include "../task_utils/task_properties.h"
#include "../open_lists/best_first_open_list.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../operator_id.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include "../utils/system.h"

#include <cstdint>
#include <unordered_set>

using namespace std;
using utils::ExitCode;

namespace enforced_hill_climbing_restarting_random_walk {
using ConstEval = const_evaluator::ConstEvaluator;
using PrefEval = pref_evaluator::PrefEvaluator;

static shared_ptr<OpenListFactory> create_ehc_open_list_factory(
    bool use_preferred, PreferredUsage preferred_usage) {
    // ConstEvaluator doens't have a non-options constructor 
    Options opts = options::Options();
    opts.set<int>("value", 1);
    shared_ptr<Evaluator> const_evaluator = make_shared<ConstEval>(opts);

    if (!use_preferred ||
        preferred_usage == PreferredUsage::PRUNE_BY_PREFERRED) {
        /*
        TODO: Reduce code duplication with search_common.cc,
        function create_standard_scalar_open_list_factory.

        It would probably make sense to add a factory function or
        constructor that encapsulates this work to the standard
        scalar open list code.
        */
        Options options;
        options.set("eval", const_evaluator);
        options.set("pref_only", false);
        return make_shared<standard_scalar_open_list::BestFirstOpenListFactory>(options);
    } else {
        /*
        TODO: Reduce code duplication with search_common.cc,
        function create_astar_open_list_factory_and_f_eval.

        It would probably make sense to add a factory function or
        constructor that encapsulates this work to the tie-breaking
        open list code.
        */
        vector<shared_ptr<Evaluator>> evals = {const_evaluator, make_shared<PrefEval>()};
        Options options;
        options.set("evals", evals);
        options.set("pref_only", false);
        options.set("unsafe_pruning", true);
        return make_shared<tiebreaking_open_list::TieBreakingOpenListFactory>(options);
    }
}

EnforcedHillClimbingSearchRRW::EnforcedHillClimbingSearchRRW(
    const Options &opts)
    : SearchEngine(opts),
    evaluator(opts.get<shared_ptr<Evaluator>>("h")),
    preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
    preferred_usage(opts.get<PreferredUsage>("preferred_usage")),
    current_eval_context(state_registry.get_initial_state(), &statistics),
    current_phase_start_g(-1),
    num_ehc_phases(0),
    last_num_expanded(-1),
    restart_strategy(opts.get<shared_ptr<RestartStrategy>>("restart")),
    probability_preferred(opts.get<double>("pref_prob")),
    rng(utils::parse_rng_from_options(opts)) {
    
    utils::g_log << "----" << endl;
    utils::g_log << "--------" << endl;
    utils::g_log << "----------------" << endl;
    utils::g_log << "--------------------------------" << endl;
    utils::g_log << "----------------" << endl;
    utils::g_log << "--------" << endl;
    utils::g_log << "----" << endl;
    utils::g_log << "rng-random_seed: " << opts.get<int>("random_seed") << endl;
    // utils::g_log << "Restart Strategy = " << restart_strategy.get() << endl;
    utils::g_log << "Use Preferred =  " << use_preferred << endl;
    for (const shared_ptr<Evaluator> &eval : preferred_operator_evaluators) {
        eval->get_path_dependent_evaluators(path_dependent_evaluators);
    }
    evaluator->get_path_dependent_evaluators(path_dependent_evaluators);

    const GlobalState &initial_state = state_registry.get_initial_state();
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_initial_state(initial_state);
    }
    use_preferred = find(preferred_operator_evaluators.begin(),
                        preferred_operator_evaluators.end(), evaluator) !=
        preferred_operator_evaluators.end();

    open_list = create_ehc_open_list_factory(
    use_preferred, preferred_usage)->create_edge_open_list();

    if (preferred_usage == PreferredUsage::RANK_PREFERRED_FIRST) {
        utils::g_log << "ILOGICAL: Ranking by preferred does nothing because of randomizing operators" << endl;
        utils::g_log << "Ranking only 'influences' hill-climbing (does not prune as all ops are randomized anyway) but is not used in local minima" << endl;
        utils::g_log << "TODO: i think this is actually fine (should not exit) bc RANK is fine within EHC, but won't have effect in UHR" << endl;
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    }

    if (!use_preferred && probability_preferred != -1) {
        utils::g_log << "ILOGICAL: preferred operator heuristic NOT specified but wanting to use a biased probability distribution" << endl;
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    }
}

EnforcedHillClimbingSearchRRW::~EnforcedHillClimbingSearchRRW() {
    // delete g_evaluator;
    // delete open_list;
}

void EnforcedHillClimbingSearchRRW::reach_state(
    const GlobalState &parent, OperatorID op_id, const GlobalState &state) {
    // for (shared_ptr<Heuristic>heur : heuristics) {
    //     heur->reach_state(parent, op, state);
    // }
    // should it be this or preferred_operator_evaluators?
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_state_transition(parent, op_id, state);
    }
}

// void EnforcedHillClimbingSearchRRW::save_plan_if_necessary() {
//     if (found_solution()) {
//         plan_manager.save_plan(plan, task_proxy);
//     }
// }

void EnforcedHillClimbingSearchRRW::initialize() {
    assert(evaluator);
    utils::g_log << "Conducting enforced hill-climbing search, (real) bound = "
        << bound << endl;
    if (use_preferred) {
        utils::g_log << "Using preferred operators for "
            << (preferred_usage == PreferredUsage::RANK_PREFERRED_FIRST ?
            "ranking successors" : "pruning") << endl;
    }

    bool dead_end = current_eval_context.is_evaluator_value_infinite(evaluator.get());
    statistics.inc_evaluated_states();
    print_initial_evaluator_values(current_eval_context);

    if (dead_end) {
        utils::g_log << "Initial state is a dead end, no solution" << endl;
        if (evaluator->dead_ends_are_reliable())
            utils::exit_with(ExitCode::SEARCH_UNSOLVABLE);
        else
            utils::exit_with(ExitCode::SEARCH_UNSOLVED_INCOMPLETE);
    }

    // Re-initialize
    solution_found = false;
    plan.clear();
    d_counts.clear();

    SearchNode node = search_space.get_node(current_eval_context.get_state());
    node.open_initial();

    num_ehc_phases = 0;
    current_phase_start_g = 0;  // this was not present before
}

void EnforcedHillClimbingSearchRRW::insert_successor_into_open_list(
    const EvaluationContext &eval_context,
    int parent_g,
    OperatorID op_id,
    bool preferred) {
    OperatorProxy op = task_proxy.get_operators()[op_id];
    int succ_g = parent_g + get_adjusted_cost(op);
    EdgeOpenListEntry entry = make_pair(
        eval_context.get_state().get_id(), op_id);
    EvaluationContext new_eval_context(
        eval_context.get_cache(), succ_g, preferred, &statistics);
    open_list->insert(new_eval_context, entry);
    statistics.inc_generated_ops();
}

void EnforcedHillClimbingSearchRRW::expand(EvaluationContext &eval_context) {
    // SearchNode node = search_space.get_node(eval_context.get_state());
    // int node_g = node.get_g();

    // ordered_set::OrderedSet<OperatorID> ops;
    // get_biased_successors(eval_context, ops);
    // statistics.inc_expanded();
    // statistics.inc_generated_ops(ops.size());

    // node.close();
    // above is the old code (which is unfinished)

    SearchNode node = search_space.get_node(eval_context.get_state());
    int node_g = node.get_g();

    ordered_set::OrderedSet<OperatorID> preferred_operators;
    if (use_preferred) {
        for (const shared_ptr<Evaluator> &preferred_operator_evaluator : preferred_operator_evaluators) {
            collect_preferred_operators(eval_context,
                                        preferred_operator_evaluator.get(),
                                        preferred_operators);
        }
    }

    if (use_preferred && preferred_usage == PreferredUsage::PRUNE_BY_PREFERRED) {
        for (OperatorID op_id : preferred_operators) {
            insert_successor_into_open_list(
                eval_context, node_g, op_id, true);
        }
    } else { //if (use_preferred && preferred_usage == PreferredUsage::RANK_BY_PREFERRED) {
        /* The successor ranking implied by RANK_BY_PREFERRED is done
           by the open list. */
        vector<OperatorID> successor_operators;
        successor_generator.generate_applicable_ops(
            eval_context.get_state(), successor_operators);
        for (OperatorID op_id : successor_operators) {
            bool preferred = use_preferred &&
                preferred_operators.contains(op_id);
            insert_successor_into_open_list(
                eval_context, node_g, op_id, preferred);
        }
    }

    statistics.inc_expanded();
    node.close();
}

void EnforcedHillClimbingSearchRRW::get_biased_successors(EvaluationContext &eval_context, ordered_set::OrderedSet<OperatorID> &ops) {
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

    // favouring to update stats after random number is generated
    // b/c, in theory, we wouldn't need to generate the unselected operator set
    // statistics.inc_expanded();
    // statistics.inc_generated_ops(pref_ops.size() + non_pref_ops.size());
	
    if (use_preferred && probability_preferred >= 0 && pref_ops.size() > 0) {
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
    // ops.shuffle(*rng);
}

bool EnforcedHillClimbingSearchRRW::check_goal_and_set_plan(const GlobalState &state) {
    if (task_properties::is_goal_state(task_proxy, state)) {
        utils::g_log << "Solution found!" << endl;
        set_plan(plan);
        return true;
    }
    return false;
}

void EnforcedHillClimbingSearchRRW::search() {
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

    utils::g_log << "Actual search time: " << timer->get_elapsed_time() << endl;
    delete timer;
}

SearchStatus EnforcedHillClimbingSearchRRW::step() {
    last_num_expanded = statistics.get_expanded();
    search_progress.check_progress(current_eval_context);

    if (check_goal_and_set_plan(current_eval_context.get_state())) {
        return SOLVED;
    }

    expand(current_eval_context);
    return ehc();
}

SearchStatus EnforcedHillClimbingSearchRRW::ehc() {
    while (!open_list->empty()) {
        EdgeOpenListEntry entry = open_list->remove_min();
        StateID parent_state_id = entry.first;
        OperatorID last_op_id = entry.second;
        OperatorProxy last_op = task_proxy.get_operators()[last_op_id];

        GlobalState parent_state = state_registry.lookup_state(parent_state_id);
        SearchNode parent_node = search_space.get_node(parent_state);

        // d: distance from initial node in this EHC phase
        int d = parent_node.get_g() - current_phase_start_g +
            get_adjusted_cost(last_op);

        if (parent_node.get_real_g() + last_op.get_cost() >= bound)
            continue;

        GlobalState state = state_registry.get_successor_state(parent_state, last_op);
        statistics.inc_generated();

        SearchNode node = search_space.get_node(state);

        if (node.is_new()) {
            EvaluationContext eval_context(state, &statistics);
            reach_state(parent_state, last_op_id, state);
            statistics.inc_evaluated_states();

            if (eval_context.is_evaluator_value_infinite(evaluator.get())) {
                node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }

            int h = eval_context.get_evaluator_value(evaluator.get());
            node.open(parent_node, last_op, get_adjusted_cost(last_op));

            if (h < current_eval_context.get_evaluator_value(evaluator.get())) {
                ++num_ehc_phases;
                if (d_counts.count(d) == 0) {
                    d_counts[d] = make_pair(0, 0);
                }
                pair<int, int> &d_pair = d_counts[d];
                d_pair.first += 1;
                d_pair.second += statistics.get_expanded() - last_num_expanded;

                current_eval_context = eval_context;
                open_list->clear();
                current_phase_start_g = node.get_g();
                plan.push_back(last_op_id);
                return IN_PROGRESS;
            } else {
                // Contribution to make plateaus RWs. do not expand child...
                // ..only consume immediate neighbors and if no appropriate state is found, do RWs
                //expand(eval_context);
            }
        }
    }
    // open list is empty..perform RWs or return no solution after enough trials

    int current_hvalue = current_eval_context.get_evaluator_value(evaluator.get());
    EvaluationContext rw_eval_context = current_eval_context;
    int hvalue = current_hvalue;
    utils::g_log << "Entering RRWs: to beat: " << hvalue << endl;
    restart_strategy->reset_sequence();

    uint64_t MAX_TIMESTEP = 1;
    MAX_TIMESTEP <<= 63;

    vector<OperatorID> actions;
    do {
        if (timer->is_expired()) {
            // This will never happen, timer handled by system
            utils::g_log << "timeout occurred during RW" << endl;
            return TIMEOUT;
        }
        uint64_t restart_length = restart_strategy->next_sequence_value();
        if (restart_length < (MAX_TIMESTEP >> 1)) {
            restart_length = restart_length << 1;	// scale by 2 because we know depth 1 successors are no good
        }
        uint64_t timestep = 0;
        rw_eval_context = current_eval_context;
        actions.clear();
        while (hvalue >= current_hvalue && timestep < restart_length)
        {
            //utils::g_log << eval_context.get_state().get_id() << " -> " << endl;
            ordered_set::OrderedSet<OperatorID> ops;
            get_biased_successors(rw_eval_context, ops);
            if (ops.size() == 0) {
                utils::g_log << "Pruned all operators -- doing a pseduo-restart" << endl;
                rw_eval_context = current_eval_context;
                actions.clear();
            }
            else {
                // randomly select op
                int random_op_id_index = (*rng)(ops.size());
                OperatorID random_op_id = ops[random_op_id_index];
                OperatorProxy random_op = task_proxy.get_operators()[random_op_id];
                GlobalState state = state_registry.get_successor_state(rw_eval_context.get_state(), random_op);

                //reach_state(rw_eval_context.get_state(), *random_op, state);

                rw_eval_context = EvaluationContext(state, &statistics);	// Eval Context of successor
                hvalue = rw_eval_context.get_evaluator_value(evaluator.get()); // hvalue of successor
                statistics.inc_evaluated_states();	// Evaluating random state
                statistics.inc_expanded();	// Expanding current state
                statistics.inc_generated();	// Only generating one (random) state
                statistics.inc_generated_ops(ops.size());
                // should inc_expanded_states() or inc_generated_states()?
                actions.push_back(random_op_id);
            }
            ++timestep;
        }
        //utils::g_log << rw_eval_context.get_state().get_id() << "(" << hvalue << ")" << endl << "---" << endl;
    } while (hvalue >= current_hvalue);

    // found escape, update the new state for ehc to continue
    // update plan by adding the RW op sequence
    utils::g_log << "Plan size is becoming: " <<  plan.size() << " + " << actions.size() << " = ";
    plan.insert(std::end(plan), std::begin(actions), std::end(actions));
    utils::g_log << plan.size() << endl;
    current_eval_context = rw_eval_context;
    return IN_PROGRESS;
}

void EnforcedHillClimbingSearchRRW::print_statistics() const {
    statistics.print_detailed_statistics();
	utils::g_log << "Termination Status: " << search_status::getStringFromSearchStatus(status) << "." << endl;

    utils::g_log << "EHC phases: " << num_ehc_phases << endl;
    assert(num_ehc_phases != 0);
    utils::g_log << "Average expansions per EHC phase: "
                 << static_cast<double>(statistics.get_expanded()) / num_ehc_phases
                 << endl;

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
    parser.document_synopsis("Lazy enforced hill-climbing with RRW in plateaus", "");
    parser.add_option<shared_ptr<Evaluator>>("h", "heuristic");
    vector<string> preferred_usages;
    preferred_usages.push_back("DO_NOTHING");
    preferred_usages.push_back("PRUNE_BY_PREFERRED");
    preferred_usages.push_back("RANK_PREFERRED_FIRST");
    parser.add_enum_option<PreferredUsage>(
        "preferred_usage",
        preferred_usages,
        "preferred operator usage",
        "DO_NOTHING"
    );
    parser.add_list_option<shared_ptr<Evaluator>>(
        "preferred",
        "use preferred operators of these heuristics",
        "[]");

    parser.add_option<shared_ptr<RestartStrategy>>("restart", "restart strategy (luby [default] or exponential)", "luby");
    parser.add_option<double>("pref_prob", "probability of selecting a preferred operator in local minima (to create distributions. 0 means zero probability of selecting a preferred operator. -1 means do not add any bias (treat pref and unpref the same). Valid input is [0,100])", "-1");

    utils::add_rng_options(parser);
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<EnforcedHillClimbingSearchRRW>(opts);
}

static Plugin<SearchEngine> _plugin("ehc_rrw", _parse);
}