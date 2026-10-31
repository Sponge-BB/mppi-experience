"""Pure, strict transformation of the frozen native L-BFGS header for SIL.

The result uses namespace ``domain_lbfgs`` and its own include guard, so it may
coexist with the unmodified native implementation. No files are changed here.
Only typed finite static-domain trial failures are recoverable. The initial
evaluation and unrelated exceptions propagate unchanged. Callers MUST reject
negative solver returns before consuming mutable objective/trajectory caches:
restoring numerical x/g/f does not restore callback-owned data.
"""

_TRIAL_EVALUATION = """            /* Evaluate the function and gradient values. */
            f = cd.proc_evaluate(cd.instance, x, g);
            ++count;

"""

_TRIAL_CHECKS = """            /* Test for errors. */
            if (std::isinf(f) || std::isnan(f))
            {
                return LBFGSERR_INVALID_FUNCVAL;
            }
            /* Check the Armijo condition. */
            if (f > finit + stp * dgtest)
            {
                nu = stp;
                brackt = true;
            }
            else
            {
                /* Check the weak Wolfe condition. */
                if (g.dot(s) < dstest)
                {
                    mu = stp;
                }
                else
                {
                    return count;
                }
            }
"""

_REJECTION_EVALUATION = """            /* A rejected trial has no objective value or usable gradient. */
            bool rejected_trial = false;
            try
            {
                f = cd.proc_evaluate(cd.instance, x, g);
            }
            catch (const domain_step::RejectedTrial &)
            {
                domain_step::recordRejected();
                had_domain_rejection = true;
                rejected_trial = true;
            }
            ++count;

            if (rejected_trial)
            {
                nu = stp;
                brackt = true;
            }
            else
            {
""" + "".join("    " + line if line.strip() else line
              for line in _TRIAL_CHECKS.replace(
                  "                    return count;",
                  "                    if (had_domain_rejection)\n"
                  "                        domain_step::recordAcceptedAfterRejection();\n"
                  "                    return count;").splitlines(keepends=True)) + "            }\n"

_PATCHES = (
    ("private include guard", "#ifndef LBFGS_HPP\n#define LBFGS_HPP",
     "#ifndef DOMAIN_STEP_LBFGS_HPP\n#define DOMAIN_STEP_LBFGS_HPP"),
    ("typed signal include", "#include <algorithm>\n",
     '#include <algorithm>\n#include "domain_step_rejection.hpp"\n'),
    ("private namespace", "namespace lbfgs\n{", "namespace domain_lbfgs\n{"),
    ("namespace closing comment", "} // namespace lbfgs", "} // namespace domain_lbfgs"),
    ("trial rejection handling", _TRIAL_EVALUATION + _TRIAL_CHECKS,
     _REJECTION_EVALUATION),
    ("line-search numerical rollback", "        finit = f;\n",
     """        finit = f;
        bool had_domain_rejection = false;
        // Numerical rollback only. Callback-owned caches are NOT restored;
        // callers must reject negative returns before using those caches.
        const auto domain_failure = [&](const int reason) {
            x = xp;
            g = gp;
            f = finit;
            return reason;
        };
"""),
) + tuple(
    ("rollback " + code, "return " + code + ";",
     "return domain_failure(" + code + ");")
    for code in (
        "LBFGSERR_INVALID_FUNCVAL", "LBFGSERR_MAXIMUMLINESEARCH",
        "LBFGSERR_WIDTHTOOSMALL", "LBFGSERR_MINIMUMSTEP", "LBFGSERR_MAXIMUMSTEP",
    )
)


def transform_lbfgs(native_text):
    """Return a domain-rejecting local header; fail on drift/duplicate markers.

    Every native marker must occur exactly once in the supplied string. This
    intentionally rejects repeat application and unreviewed optimizer changes.
    All parameter values, iteration/evaluation limits and Wolfe logic remain
    native; a rejected point joins the existing upper-bracket/bisection path.
    """
    if not isinstance(native_text, str):
        raise TypeError("native L-BFGS header must be a string")
    for name, before, _ in _PATCHES:
        count = native_text.count(before)
        if count != 1:
            raise ValueError("L-BFGS marker {!r}: expected exactly one match, got {}"
                             .format(name, count))
    text = native_text
    for _, before, after in _PATCHES:
        if text.count(before) != 1:
            raise ValueError("L-BFGS transformation markers overlap unexpectedly")
        text = text.replace(before, after, 1)
    return text
