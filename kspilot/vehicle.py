"""Vehicle structure helpers: core (recovery) parts, staging protection, payload."""


def core_parts(vessel):
    """Parts that are never decoupled — the stage that flies home."""
    return [p for p in vessel.parts.all if p.decouple_stage == -1]


def _subtree(part):
    out, todo = [], list(part.children)
    while todo:
        p = todo.pop()
        out.append(p)
        todo.extend(p.children)
    return out


def payload_stage(vessel):
    """Stage whose decoupler releases a payload from the core, or None.

    A decoupler mounted on the core is a *booster* mount if anything it
    drops has an engine that ignites before the decoupler fires; otherwise
    it releases a payload.
    """
    best = None
    for d in vessel.parts.decouplers:
        p = d.part
        if p.stage < 0 or p.parent is None or p.parent.decouple_stage != -1:
            continue
        dropped = _subtree(p) if p.children else []
        booster = any(q.engine is not None and q.stage > p.stage for q in dropped)
        if not booster:
            best = p.stage if best is None else max(best, p.stage)
    return best


def autostage_limit(vessel):
    """Lowest stage the ascent may activate (payload release is a separate, explicit step)."""
    ps = payload_stage(vessel)
    return ps + 1 if ps is not None else 0


def clear_pad(sc, keep, log=print, timeout=5.0):
    """Remove launch-clamp debris left standing on the pad (AutoLab bench command, no scene change)."""
    import os
    import time
    from . import core
    done = os.path.join(core._AL, "cmd.done")
    if os.path.exists(done):
        os.remove(done)
    with open(os.path.join(core._AL, "cmd.req"), "w") as f:
        f.write("clearpad")
    t0 = time.time()
    while time.time() - t0 < timeout and not os.path.exists(done):
        time.sleep(0.1)
    msg = open(done).read().strip() if os.path.exists(done) else "clearpad: no answer"
    log("pad cleared: " + msg)
