import random

from ttt.game import HUMAN, ROBOT, Game, choose_move, empty_cells, winner


def play(mode, human_policy, seed):
    g = Game(mode, seed=seed)
    rng = random.Random(seed + 1000)
    while g.result is None:
        g.human_move(human_policy(g.board, rng))
        if g.result is None:
            g.robot_move()
    return g.result


def random_human(board, rng):
    return rng.choice(empty_cells(board))


def greedy_human(board, rng):
    """Takes a win if one exists, else blocks, else random - like a sharp student."""
    for who in (HUMAN, ROBOT):
        for i in empty_cells(board):
            b = list(board)
            b[i] = who
            if winner(b) == who:
                return i
    return rng.choice(empty_cells(board))


def test_winner():
    assert winner(["X", "X", "X"] + [None] * 6) == "X"
    assert winner(["X", "O", "X", "X", "O", "O", "O", "X", "X"]) == "draw"
    assert winner([None] * 9) is None


def test_hard_never_loses():
    for seed in range(300):
        assert play("hard", random_human, seed) != HUMAN
        assert play("hard", greedy_human, seed) != HUMAN


def test_robot_takes_win_and_blocks():
    # robot can win at index 2
    b = ["O", "O", None, "X", "X", None, "X", None, None]
    assert choose_move(b, "hard")[0] == 2
    # robot must block X at index 5
    b = ["O", None, None, "X", "X", None, None, None, None]
    assert choose_move(b, "hard")[0] == 5


def test_easy_and_gift_are_beatable():
    easy = sum(play("easy", greedy_human, s) == HUMAN for s in range(300))
    gift = sum(play("gift", greedy_human, s) == HUMAN for s in range(300))
    print(f"student wins vs easy: {easy}/300, vs gift: {gift}/300")
    assert 0 < easy < 300
    assert gift > easy
