#include <quantum/coaster/ForceSection.hpp>
#include <quantum/coaster/GeometricSection.hpp>

#include <stdexcept>

namespace quantum::coaster
{
    namespace
    {
        void validateScalarTransitionDefinition(
            const math::ScalarTransition& transition
        )
        {
            // The scalar evaluator remains authoritative for domain and
            // authored-value validation. A normalized interior evaluation
            // additionally rejects unsupported transition types even when no
            // representable scalar-domain midpoint exists.
            static_cast<void>(math::evaluateScalarTransition(
                transition,
                transition.domainBegin
            ));
            static_cast<void>(math::evaluateTransition(
                transition.transitionType,
                0.5
            ));
        }

        void requireMatchingDomain(
            const math::ScalarTransition& reference,
            const math::ScalarTransition& channel
        )
        {
            if (channel.domainBegin != reference.domainBegin
                || channel.domainEnd != reference.domainEnd)
            {
                throw std::invalid_argument(
                    "All authored section channels must use exactly the same domain."
                );
            }
        }
    }

    void validateForceSection(const ForceSection& section)
    {
        validateScalarTransitionDefinition(section.verticalForce);
        validateScalarTransitionDefinition(section.lateralForce);
        validateScalarTransitionDefinition(section.roll);

        requireMatchingDomain(section.verticalForce, section.lateralForce);
        requireMatchingDomain(section.verticalForce, section.roll);
    }

    void validateGeometricSection(const GeometricSection& section)
    {
        validateScalarTransitionDefinition(section.pitch);
        validateScalarTransitionDefinition(section.yaw);
        validateScalarTransitionDefinition(section.roll);

        requireMatchingDomain(section.pitch, section.yaw);
        requireMatchingDomain(section.pitch, section.roll);
    }

    GeometricSectionState evaluateGeometricSection(
        const GeometricSection& section,
        const double independentValue
    )
    {
        validateGeometricSection(section);

        return {
            math::evaluateScalarTransition(section.pitch, independentValue),
            math::evaluateScalarTransition(section.yaw, independentValue),
            math::evaluateScalarTransition(section.roll, independentValue)
        };
    }
}
