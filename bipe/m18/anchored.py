"""M18 phase E (experimental, not used by any shipped policy): PPO with an anchor to a
frozen teacher policy.

Result (17 November 2026): two fidelity fine-tunes anchored to fine-tune 1 at weights 1
and 5 kept the mean action within an MSE of about 5e-4 of the teacher yet lost more on
the full plant than the plain fine-tune (docs/milestones/18-sim-to-real-plan.md), so a
per-action anchor does not hold closed-loop behaviour. Kept as a documented tool.

M15 and M16 showed that a fine-tune aimed at new commands forgets the old ones
(fine-tune 3 lost all ten low pushes). This algorithm adds a behaviour-cloning term to
PPO's loss, coef x mean((mu_student(o) - mu_teacher(o))^2) over every sampled
observation, so the policy moves only where the new rewards pull it hard enough.

Selected with mjlab's ``--agent.algorithm.class-name bipe.m18.anchored:AnchoredPPO``;
the teacher and weight come from the environment, since mjlab's algorithm config has
no free fields:

    BIPE_ANCHOR_CHECKPOINT  the teacher's checkpoint (an rsl_rl model_*.pt)
    BIPE_ANCHOR_COEF        the weight (default 1.0)

The term is added without copying rsl_rl's update: just before each mini-batch's
``loss.backward()`` PPO zeroes the gradients, and the wrapped ``zero_grad`` then
back-propagates the anchor on that mini-batch, so the two gradients add and are
clipped and applied together. Written against rsl-rl-lib 5.4.2 (requirements-rl.lock).
"""

from __future__ import annotations

import copy
import os

import torch
from rsl_rl.algorithms import PPO


class AnchoredPPO(PPO):
    def __init__(self, actor, critic, storage, **kwargs):
        super().__init__(actor, critic, storage, **kwargs)
        path = os.environ.get("BIPE_ANCHOR_CHECKPOINT")
        if not path:
            raise ValueError("AnchoredPPO needs BIPE_ANCHOR_CHECKPOINT")
        self.anchor_coef = float(os.environ.get("BIPE_ANCHOR_COEF", "1.0"))
        state = torch.load(path, map_location=self.device, weights_only=False)
        self.teacher = copy.deepcopy(actor)
        self.teacher.load_state_dict(state["actor_state_dict"])
        self.teacher.eval()
        for parameter in self.teacher.parameters():
            parameter.requires_grad_(False)
        self._batch = None
        self._anchor_sum = 0.0
        self._anchor_count = 0

    def anchor_loss(self, obs) -> torch.Tensor:
        with torch.no_grad():
            target = self.teacher(obs)
        return (self.actor(obs) - target).pow(2).mean()

    def update(self) -> dict[str, float]:
        storage, optimizer = self.storage, self.optimizer
        generators = storage.mini_batch_generator, storage.recurrent_mini_batch_generator
        zero_grad = optimizer.zero_grad

        def capture(make):
            def wrapped(*args, **kwargs):
                for batch in make(*args, **kwargs):
                    self._batch = batch
                    yield batch

            return wrapped

        def anchored_zero_grad(*args, **kwargs):
            zero_grad(*args, **kwargs)
            if self._batch is None or self.anchor_coef == 0:
                return
            n = self._batch.observations.batch_size[0]
            loss = self.anchor_loss(self._batch.observations[:n])
            (self.anchor_coef * loss).backward()
            self._anchor_sum += loss.item()
            self._anchor_count += 1

        storage.mini_batch_generator = capture(generators[0])
        storage.recurrent_mini_batch_generator = capture(generators[1])
        optimizer.zero_grad = anchored_zero_grad
        self._anchor_sum, self._anchor_count = 0.0, 0
        try:
            losses = super().update()
        finally:
            storage.mini_batch_generator, storage.recurrent_mini_batch_generator = generators
            optimizer.zero_grad = zero_grad
            self._batch = None
        losses["anchor"] = self._anchor_sum / max(self._anchor_count, 1)
        return losses
