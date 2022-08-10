from matplotlib import pyplot as plt


def plot(t, x, u):

    plt.subplot(2, 1, 1)
    ax = plt.gca()

    ax.annotate('$t$', xy=(1., -0.06), ha='left', va='top', xycoords='axes fraction')
    ax.annotate('$u_1(t),~u_2(t)$', xy=(0.0, 1.12), xytext=(-15, 2), ha='left',
                va='top', xycoords='axes fraction',
                textcoords='offset points')
    plt.title('Оптимальное управление')
    # plt.title('Optimal Control')
    plt.grid(True)
    plt.xticks(list(range(0, 22, 2)))
    plt.plot(t, u[:, 0], label='$u_1(t)$')
    plt.plot(t, u[:, 1], label='$u_2(t)$')
    plt.legend()

    plt.subplot(2, 1, 2)
    ax = plt.gca()
    ax.annotate(r'$\xi(t)$', xy=(0.98, -0.06), ha='left', va='top',
                xycoords='axes fraction')
    ax.annotate(r'$\eta(t)$', xy=(0.0, 1.12), xytext=(-15, 2), ha='left',
                va='top', xycoords='axes fraction',
                textcoords='offset points')
    plt.title('Фазовый портрет')
    # plt.title('Phase Portrait')
    plt.grid(True)
    plt.plot(x[:, 0], x[:, 2])

    # plt.style.use('grayscale')
    plt.subplots_adjust(hspace=0.5)
    plt.show()
