"""Tic-tac-toe engine. Pure Python, no ROS - easy to unit test.

Board: list of 9 cells, index 0..8 row by row, values "X", "O" or None.
Student plays X and always starts; the robot plays O.
Cell numbers shown to people and used in poses are 1..9 (index + 1).
"""
from __future__ import annotations

import random
from functools import lru_cache

LINES = [(0, 1, 2), (3, 4, 5), (6, 7, 8),
         (0, 3, 6), (1, 4, 7), (2, 5, 8),
         (0, 4, 8), (2, 4, 6)]

HUMAN, ROBOT = "X", "O"

# Operator-selectable difficulty.
#   mistake_rate: chance per robot move of playing a non-best move
#   max_mistakes: cap per game, so an "easy" robot still looks clever
MODES = {
    "hard": dict(mistake_rate=0.0, max_mistakes=0),
    "easy": dict(mistake_rate=0.5, max_mistakes=1),
    "gift": dict(mistake_rate=1.0, max_mistakes=2),   # student should win
}


def winner(board) -> str | None:
    """'X', 'O', 'draw', or None if the game is still running."""
    for a, b, c in LINES:
        if board[a] and board[a] == board[b] == board[c]:
            return board[a]
    return "draw" if all(board) else None


def winning_line(board) -> tuple | None:
    for line in LINES:
        a, b, c = line
        if board[a] and board[a] == board[b] == board[c]:
            return line
    return None


def empty_cells(board) -> list[int]:
    return [i for i, v in enumerate(board) if v is None]


@lru_cache(maxsize=None)
def _minimax(state: tuple, to_move: str) -> int:
    """Score from the robot's view: +10-depth win, 0 draw, -10+depth loss."""
    w = winner(state)
    depth = sum(1 for v in state if v)
    if w == ROBOT:
        return 10 - depth
    if w == HUMAN:
        return depth - 10
    if w == "draw":
        return 0
    nxt = HUMAN if to_move == ROBOT else ROBOT
    scores = []
    for i in empty_cells(state):
        s = list(state)
        s[i] = to_move
        scores.append(_minimax(tuple(s), nxt))
    return max(scores) if to_move == ROBOT else min(scores)


def move_scores(board) -> dict[int, int]:
    """Score of every empty cell for the robot. Shown on the big screen."""
    out = {}
    for i in empty_cells(board):
        s = list(board)
        s[i] = ROBOT
        out[i] = _minimax(tuple(s), HUMAN)
    return out


@lru_cache(maxsize=None)
def _count_futures(state: tuple, to_move: str) -> int:
    if winner(state):
        return 1
    nxt = HUMAN if to_move == ROBOT else ROBOT
    total = 0
    for i in empty_cells(state):
        s = list(state)
        s[i] = to_move
        total += _count_futures(tuple(s), nxt)
    return total


def futures_checked(board) -> int:
    """How many complete games the robot considered for this move (for the screen)."""
    return _count_futures(tuple(board), ROBOT)


def outcome_label(score: int) -> str:
    return "win" if score > 0 else "lose" if score < 0 else "draw"


def choose_move(board, mode: str = "hard", mistakes_so_far: int = 0,
                rng: random.Random | None = None) -> tuple[int, dict[int, int], bool]:
    """Pick the robot's move.

    Returns (index, scores, was_mistake). A mistake is a move scoring below
    the best one; if every move scores the same, no mistake is possible.
    """
    rng = rng or random
    cfg = MODES[mode]
    scores = move_scores(board)
    best = max(scores.values())
    best_moves = [i for i, s in scores.items() if s == best]
    worse = [i for i, s in scores.items() if s < best]

    if worse and mistakes_so_far < cfg["max_mistakes"] and rng.random() < cfg["mistake_rate"]:
        # "gift" picks the most helpful blunder (worst score); others pick a random one
        if mode == "gift":
            worst = min(scores[i] for i in worse)
            return rng.choice([i for i in worse if scores[i] == worst]), scores, True
        return rng.choice(worse), scores, True
    return rng.choice(best_moves), scores, False


class Game:
    """One game's state. The game node owns one of these."""

    def __init__(self, mode: str = "hard", seed: int | None = None):
        self.mode = mode
        self.rng = random.Random(seed)
        self.reset()

    def reset(self):
        self.board = [None] * 9
        self.mistakes = 0
        self.last_scores: dict[int, int] = {}
        self.turn = HUMAN

    @property
    def result(self):
        return winner(self.board)

    def human_move(self, i: int) -> bool:
        if self.result or self.turn != HUMAN or self.board[i] is not None:
            return False
        self.board[i] = HUMAN
        self.turn = ROBOT
        return True

    def robot_move(self) -> int | None:
        plan = self.plan_robot()
        if plan is None:
            return None
        self.apply_robot(*plan)
        return plan[0]

    def plan_robot(self) -> tuple[int, bool] | None:
        """Decide the robot's move without placing it (the arm must tap first)."""
        if self.result or self.turn != ROBOT:
            return None
        i, self.last_scores, mistake = choose_move(self.board, self.mode, self.mistakes, self.rng)
        return i, mistake

    def apply_robot(self, i: int, mistake: bool = False) -> None:
        self.mistakes += mistake
        self.board[i] = ROBOT
        self.turn = HUMAN

    def line_cells(self) -> list[int] | None:
        line = winning_line(self.board)
        return [i + 1 for i in line] if line else None

    def scores_for_screen(self) -> dict[str, str]:
        """{"5": "win", "1": "draw", ...} for the robot's last evaluation."""
        return {str(i + 1): outcome_label(s) for i, s in self.last_scores.items()}

    def to_dict(self) -> dict:
        return dict(board=self.board, turn=self.turn, result=self.result,
                    line=winning_line(self.board), mode=self.mode,
                    scores={str(k): v for k, v in self.last_scores.items()})
