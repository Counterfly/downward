#include "successor_generator.h"

#include "successor_generator_factory.h"
#include "successor_generator_internals.h"

#include "../abstract_task.h"
#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../global_state.h"
#include "../utils/logging.h"

using namespace std;

namespace successor_generator {
SuccessorGenerator::SuccessorGenerator(const TaskProxy &task_proxy)
    : root(SuccessorGeneratorFactory(task_proxy).create()) {
}

SuccessorGenerator::~SuccessorGenerator() = default;

void SuccessorGenerator::generate_applicable_ops(
    const State &state, vector<OperatorID> &applicable_ops) const {
    root->generate_applicable_ops(state, applicable_ops);
}

void SuccessorGenerator::generate_applicable_ops(
    const GlobalState &state, vector<OperatorID> &applicable_ops) const {
    root->generate_applicable_ops(state, applicable_ops);
}

void SuccessorGenerator::get_partitioned_operators(
    EvaluationContext &eval_context,
    const std::vector<std::shared_ptr<Evaluator>> &preferred_operator_evaluators,
    unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> &preferred_ops,
    unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> &non_preferred_ops) {
    vector<OperatorID> ops;
    root->generate_applicable_ops(eval_context.get_state(), ops);

    // utils::g_log << "Eval context: " << eval_context.get_state().get_id().get_value() << endl;
	unordered_set<int> *op_ids = nullptr;
    unordered_set<int> pref_op_ids;
    unordered_set<int> dead_end_op_ids;
    for (shared_ptr<Evaluator> pref_evaluator : preferred_operator_evaluators) {
        // if (eval_context.is_evaluator_value_infinite(pref_evaluator.get())) {
        //     // HERE:NOTE: this is-inf check wasn't being done in thesis
        //     // utils::g_log << "pref heur (" << pref_evaluator->get_description() << ") is infinite" << endl;
        //     op_ids = &dead_end_op_ids;
        // }
        // else {
        //     // utils::g_log << "pref heur (" << pref_evaluator->get_description() << ") is finite" << endl;
        //     op_ids = &pref_op_ids;
        // }
        const vector<OperatorID> &pref_ops1 = eval_context.get_preferred_operators(pref_evaluator.get());
        // utils::g_log << "pref heur = " << pref_evaluator->get_description() << " num pref ops = " << pref_ops1.size() << endl;
        for (OperatorID op : pref_ops1) {
            pref_op_ids.insert(op.get_index());
        }
    }
    // utils::g_log << "FInished pref evaluation with sizes: " << pref_op_ids.size() << " v " << dead_end_op_ids.size() << endl;

    for (OperatorID op : ops) {
        int index = op.get_index();
        if (pref_op_ids.find(index) == pref_op_ids.end()) {
            if (dead_end_op_ids.find(index) == dead_end_op_ids.end()) {
                // utils::g_log << "Could not find OperatorID = " << index << " in pref-ops or dead-end-ops, so adding op to non-pref-ops" << endl;
                non_preferred_ops.insert(op);
            }
            else {
                // utils::g_log << "OperatorID = " << index << " results in a dead-end" << endl;
            }
		}
        else {
            // now add OperatorID instances to `preferred_ops`
            // utils::g_log << "OperatorID = " << index << " is a Pref OP!!" << endl;
            preferred_ops.insert(op);
        }
    }
}

PerTaskInformation<SuccessorGenerator> g_successor_generators;
}
