#ifndef TASK_UTILS_SUCCESSOR_GENERATOR_H
#define TASK_UTILS_SUCCESSOR_GENERATOR_H

#include "../per_task_information.h"

#include <memory>
#include <unordered_set>
#include <vector>

class GlobalState;
class OperatorID;
namespace utils {
    namespace operator_id_hasher {
        struct OperatorIDHasher;
    }
};
class State;
class TaskProxy;
class Evaluator;
class EvaluationContext;

namespace successor_generator {
class GeneratorBase;

class SuccessorGenerator {
    std::unique_ptr<GeneratorBase> root;

public:
    explicit SuccessorGenerator(const TaskProxy &task_proxy);
    /*
      We cannot use the default destructor (implicitly or explicitly)
      here because GeneratorBase is a forward declaration and the
      incomplete type cannot be destroyed.
    */
    ~SuccessorGenerator();

    void generate_applicable_ops(
        const State &state, std::vector<OperatorID> &applicable_ops) const;
    // Transitional method, used until the search is switched to the new task interface.
    void generate_applicable_ops(
        const GlobalState &state, std::vector<OperatorID> &applicable_ops) const;

    // generates two sets (preferred and non-preferred) operators
    void get_partitioned_operators(
        EvaluationContext &eval_context,
        const std::vector<std::shared_ptr<Evaluator>> &preferred_operator_evaluators,
        std::unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> &preferred_ops,
        std::unordered_set<OperatorID, utils::operator_id_hasher::OperatorIDHasher> &non_preferred_ops);
    
};

extern PerTaskInformation<SuccessorGenerator> g_successor_generators;
}

#endif
