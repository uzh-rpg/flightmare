import seaborn as sns
import pandas as pd
import matplotlib.animation as animation
import matplotlib.pyplot as plt
#
sns.set_style("whitegrid")
#


def plot_reward(save_dir, varname, ylabel, save_fig=False):
    fig, ax = plt.subplots(1, 1, figsize=(7, 4))
    #
    sns.lineplot(data=data[varname].dropna(), ax=ax)
    plt.xlabel("Training Iterations")
    plt.title(ylabel + " (ppo)")
    if save_fig:
        plt.savefig(save_dir + "/" + ylabel + ".png")


if __name__ == "__main__":
    logger_dir = "./saved/2020-09-22-13-46-21/"
    ppo_var_names = ["ep_reward_mean", "ep_len_mean", "policy_entropy"]
    ppo_y_labels = ["Reward", "Episode Length", "Policy Entropy"]
    #
    sac_var_names = ["train_reward", "test_reward", "entropy"]
    sac_y_labels = ["Train Reward", "Test Reward", "Entropy"]
    #
    csv_path = logger_dir + "progress.csv"
    data = pd.read_csv(csv_path)
    for i in range(len(ppo_var_names)):
        plot_reward(
            logger_dir, varname=ppo_var_names[i], ylabel=ppo_y_labels[i])
    plt.show()
