import optuna
from bo import run_bo_multi_variable


def optuna_objective(trial):
    # Let Optuna suggest BO parameters
    budget = trial.suggest_int("budget", 10, 100)

    # Use your actual scenario & variables
    best_params, best_area = run_bo_multi_variable(
        scenario_path="scenarios/DEU_Flensburg-94_1_T-1.xml",
        decision_variables=[("ego", "velocity")],
        budget=budget,
        a_ref=1.0,
    )

    return (sum(best_area) - 1.0) ** 2


if __name__ == "__main__":
    study = optuna.create_study(direction="minimize")
    study.optimize(optuna_objective, n_trials=30)
    print("\nBest BO parameters found:")
    print(f"Max Iterations: {study.best_params['budget']}")
