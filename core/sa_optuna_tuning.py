import optuna
from sa import run_sa_with_scipy


def optuna_objective(trial):
    # Let Optuna suggest SA parameters
    initial_temp = trial.suggest_float("initial_temp", 1000.0, 5000.0)
    max_iter = trial.suggest_int("max_iter", 5, 12)

    # Use your actual scenario & variables
    best_params, best_area = run_sa_with_scipy(
        scenario_path="scenarios/DEU_Flensburg-94_1_T-1.xml",
        decision_variables=[("ego", "velocity")],
        max_iter=max_iter,
        initial_temp=initial_temp,
        a_ref=1.0,
    )

    return (sum(best_area) - 1.0) ** 2


if __name__ == "__main__":
    study = optuna.create_study(direction="minimize")
    study.optimize(optuna_objective, n_trials=30)
    print("\nBest SA parameters found:")
    print(f"Initial Temperature: {study.best_params['initial_temp']}")
    print(f"Max Iterations: {study.best_params['max_iter']}")
