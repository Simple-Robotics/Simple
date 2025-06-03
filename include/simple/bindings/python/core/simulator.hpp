//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_python_algorithm_simulator_hpp__
#define __simple_python_algorithm_simulator_hpp__

#include "simple/bindings/python/fwd.hpp"
#include "simple/core/simulator.hpp"

#include <eigenpy/std-vector.hpp>
#include <eigenpy/copyable.hpp>

// clang-format off
#define SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(type, name, desc)                                          \
  add_property(                                                                                         \
    #name,                                                                                              \
    bp::make_function(                                                                                  \
      +[](Self & self) -> type & { return self.name(); }, bp::return_internal_reference<>()),           \
    bp::make_function(+[](Self &, const type &) -> void {                                               \
      PINOCCHIO_THROW(std::invalid_argument, "Cannot manually set " #name ". Use constructor instead.") \
    }),                                                                                                 \
    desc)
// clang-format on

namespace simple
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Simulator>
    struct SimulatorPythonVisitor : public boost::python::def_visitor<SimulatorPythonVisitor<Simulator>>
    {
      using Self = Simulator;
      using Scalar = typename Self::Scalar;

      using ModelHandle = typename Self::ModelHandle;
      using Model = typename Self::Model;
      using DataHandle = typename Self::DataHandle;
      using Data = typename Self::Data;
      using GeometryModelHandle = typename Self::GeometryModelHandle;
      using GeometryModel = typename Self::GeometryModel;
      using GeometryDataHandle = typename Self::GeometryDataHandle;
      using GeometryData = typename Self::GeometryData;
      using BilateralPointConstraintModelVector = typename Self::BilateralPointConstraintModelVector;
      using WeldConstraintModelVector = typename Self::WeldConstraintModelVector;
      using ConstraintsProblemHandle = typename Self::ConstraintsProblemHandle;
      using VectorXs = typename Self::VectorXs;
      using MatrixXs = typename Self::MatrixXs;
      using SpatialForce = typename Self::SpatialForce;
      using SpatialForceVector = typename Self::SpatialForceVector;
      using ConstraintSolverType = typename Self::ConstraintSolverType;
      using ConstraintSolverSettings = typename Self::ConstraintSolverSettings;
      using ADMMConstraintSolverSettings = typename Self::ADMMConstraintSolverSettings;
      using PGSConstraintSolverSettings = typename Self::PGSConstraintSolverSettings;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
          // -----------------------------
          // CONSTRUCTORS
          .def(bp::init<ModelHandle, DataHandle, GeometryModelHandle, GeometryDataHandle>(
            bp::args("self", "model", "data", "geom_model", "geom_data"), "Constructor"))
          .def(bp::init<ModelHandle, DataHandle, GeometryModelHandle, GeometryDataHandle, BilateralPointConstraintModelVector>(
            bp::args("self", "model", "data", "geom_model", "geom_data", "bilateral_point_constraint_models"), "Constructor"))
          .def(bp::init<ModelHandle, DataHandle, GeometryModelHandle, GeometryDataHandle, WeldConstraintModelVector>(
            bp::args("self", "model", "data", "geom_model", "geom_data", "weld_constraint_models"), "Constructor"))
          .def(bp::init<
               ModelHandle, DataHandle, GeometryModelHandle, GeometryDataHandle, BilateralPointConstraintModelVector,
               WeldConstraintModelVector>(
            bp::args("self", "model", "data", "geom_model", "geom_data", "bilateral_point_constraint_models", "weld_constraint_models"),
            "Constructor"))
          .def(bp::init<ModelHandle, GeometryModelHandle>(bp::args("self", "model", "geom_model"), "Constructor"))
          .def(bp::init<ModelHandle, GeometryModelHandle, BilateralPointConstraintModelVector>(
            bp::args("self", "model", "geom_model", "bilateral_point_constraint_models"), "Constructor"))
          .def(bp::init<ModelHandle, GeometryModelHandle, WeldConstraintModelVector>(
            bp::args("self", "model", "geom_model", "weld_constraint_models"), "Constructor"))
          .def(bp::init<ModelHandle, GeometryModelHandle, BilateralPointConstraintModelVector, WeldConstraintModelVector>(
            bp::args("self", "model", "geom_model", "bilateral_point_constraint_models", "weld_constraint_models"), "Constructor"))

          // -----------------------------
          // ATTRIBUTES
          .SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(Model, model, "Pinocchio model.")
          .SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(Data, data, "Pinocchio data.")
          .SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(GeometryModel, geom_model, "Pinocchio geometry model.")
          .SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(GeometryData, geom_data, "Pinocchio geometry data.")
          .SIMPLE_SIMULATOR_ADD_READONLY_HANDLE(ConstraintsProblem, constraints_problem, "Constraint problem.")

          // -- state
          .def_readonly("q", &Self::q, "Joints configuration of the system.")
          .def_readonly("v", &Self::v, "Joints velocity.")
          .def_readonly("tau", &Self::tau, "External joints torques.")
          .def_readonly("fext", &Self::fext, "External joints forces.")
          .def_readonly("dt", &Self::dt, "Time step used to compute next state.")

          // -- new state
          .def_readonly("qnew", &Self::qnew, "New joints configuration of the system.")
          .def_readonly("vfree", &Self::vfree, "New joints velocity as if there were no constraints.")
          .def_readonly("vnew", &Self::vnew, "New joints velocity.")
          .def_readonly("anew", &Self::anew, "New joints acceleration.")
          .def_readonly(
            "ftotal", &Self::ftotal,
            "Vector of total forces (external + constraint forces) applied on joints, expressed in the frame of each joint.")
          .def_readonly(
            "tau_total", &Self::tau_total, "Vector of total torques (given as input to step + constraint torques) applied on joints.")
          .def_readonly("tau_constraints", &Self::tau_constraints, "Vector of constraint torques.")

          // -- constraint solvers
          .def_readonly(
            "constraint_solver_type", &Self::constraint_solver_type, "Type of constraint solver used in the last call to `step`.")
          .def_readwrite("warm_start_constraints_forces", &Self::warm_start_constraints_forces, "Warm start constraint forces boolean.")
          .def_readwrite(
            "admm_constraint_solver_settings", &Self::admm_constraint_solver_settings, "Settings of the ADMM constraint solver.")
          .def_readwrite("admm_constraint_solver", &Self::admm_constraint_solver, "The ADMM constraints solver.")
          .def_readwrite("pgs_constraint_solver_settings", &Self::pgs_constraint_solver_settings, "Settings of the PGS constraint solver.")
          .def_readwrite("pgs_constraint_solver", &Self::pgs_constraint_solver, "The PGS constraints solver.")

          // -- misc
          .def_readwrite("measure_timings", &Self::measure_timings, "Measure timings of the `step` method.")

          // -----------------------------
          // METHODS
          .def(
            "reset", &Self::reset, bp::args("self"),
            "Resets the simulator to accept a new initial state, i.e. before looping on the `step` method.")
          //
          .def(
            "step",
            +[](Self & self, const VectorXs & q, const VectorXs & v, const VectorXs & tau, Scalar dt, std::size_t nsteps = 1) {
              self.template step<::pinocchio::ADMMContactSolverTpl>(q, v, tau, dt);
              for (std::size_t i = 0; i < nsteps - 1; ++i)
              {
                self.template step<::pinocchio::ADMMContactSolverTpl>(self.qnew, self.vnew, tau, dt);
              }
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("dt"), bp::arg("nsteps") = 1),
            "Do one step of simulation with default constraint solver (ADMM).")
          //
          .def(
            "rollout",
            +[](Self & self, const VectorXs & q, const VectorXs & v, const VectorXs & tau, Scalar dt, std::size_t nsteps = 1) {
              VectorXs q_ = q;
              VectorXs v_ = v;
              VectorXs tau_ = tau;
              Py_BEGIN_ALLOW_THREADS;
              self.template step<::pinocchio::ADMMContactSolverTpl>(q_, v_, tau_, dt);
              for (std::size_t i = 0; i < nsteps - 1; ++i)
              {
                self.template step<::pinocchio::ADMMContactSolverTpl>(self.qnew, self.vnew, tau, dt);
              }
              Py_END_ALLOW_THREADS;
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("dt"), bp::arg("nsteps") = 1),
            " Compute a trajectory by performing a rollout of a policy. This function releases "
            "Python GIL so it can be parallelized (input vectors are copied for safety). TODO: in "
            "the future, rollout should take a control policy as input rather than a constant "
            "torque.")
          //
          .def(
            "rollout",
            +[](Self & self, const VectorXs & q, const VectorXs & v, const std::vector<VectorXs> & taus, Scalar dt) {
              VectorXs q_ = q;
              VectorXs v_ = v;
              Py_BEGIN_ALLOW_THREADS;
              for (const auto & tau : taus)
              {
                self.template step<::pinocchio::ADMMContactSolverTpl>(q_, v_, tau, dt);
                q_ = self.qnew;
                v_ = self.vnew;
              }
              Py_END_ALLOW_THREADS;
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("taus"), bp::arg("dt")),
            "Compute a trajectory by performing a rollout of a policy with a list of taus. This function releases "
            "Python GIL so it can be parallelized (input vectors are copied for safety).")
          //
          .def(
            "rollout",
            +[](Self & self, const VectorXs & q, const VectorXs & v, const MatrixXs & taus, Scalar dt) {
              VectorXs q_ = q;
              VectorXs v_ = v;
              Py_BEGIN_ALLOW_THREADS;
              for (int i = 0; i < taus.rows(); ++i)
              {
                VectorXs tau = taus.row(i);
                self.template step<::pinocchio::ADMMContactSolverTpl>(q_, v_, tau, dt);
                q_ = self.qnew;
                v_ = self.vnew;
              }
              Py_END_ALLOW_THREADS;
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("taus"), bp::arg("dt")),
            "Compute a trajectory by performing a rollout of a policy with a matrix of taus. This function releases "
            "Python GIL so it can be parallelized (input vectors are copied for safety).")
          //
          .def(
            "step",
            +[](
               Self & self, const VectorXs & q, const VectorXs & v, const VectorXs & tau, const SpatialForceVector & fext, Scalar dt,
               std::size_t nsteps = 1) {
              self.template step<::pinocchio::ADMMContactSolverTpl>(q, v, tau, fext, dt);
              for (std::size_t i = 0; i < nsteps - 1; ++i)
              {
                self.template step<::pinocchio::ADMMContactSolverTpl>(self.qnew, self.vnew, tau, fext, dt);
              }
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("fext"), bp::arg("dt"), bp::arg("nsteps") = 1),
            "Do one step of simulation with default constraint solver (ADMM).")
          //
          .def(
            "stepPGS",
            +[](Self & self, const VectorXs & q, const VectorXs & v, const VectorXs & tau, Scalar dt, std::size_t nsteps = 1) {
              self.template step<::pinocchio::PGSContactSolverTpl>(q, v, tau, dt);
              for (std::size_t i = 0; i < nsteps - 1; ++i)
              {
                self.template step<::pinocchio::PGSContactSolverTpl>(self.qnew, self.vnew, tau, dt);
              }
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("dt"), bp::arg("nsteps") = 1),
            "Do one step of simulation with PGS constraint solver.")
          //
          .def(
            "stepPGS",
            +[](
               Self & self, const VectorXs & q, const VectorXs & v, const VectorXs & tau, const SpatialForceVector & fext, Scalar dt,
               std::size_t nsteps = 1) {
              self.template step<::pinocchio::PGSContactSolverTpl>(q, v, tau, fext, dt);
              for (std::size_t i = 0; i < nsteps - 1; ++i)
              {
                self.template step<::pinocchio::PGSContactSolverTpl>(self.qnew, self.vnew, tau, fext, dt);
              }
            },
            (bp::arg("self"), bp::arg("q"), bp::arg("v"), bp::arg("tau"), bp::arg("fext"), bp::arg("dt"), bp::arg("nsteps") = 1),
            "Do one step of simulation with PGS constraint solver.")

          .def("getStepCPUTimes", &Self::getStepCPUTimes, bp::args("self"), "Get timings of the last call to the `step` method.")
          //
          .def(
            "getConstraintSolverCPUTimes", &Self::getConstraintSolverCPUTimes, bp::args("self"),
            "Get timings of the call to the constraint solver in the last call to the `step` method. "
            "These timings can be 0 if the system has no constraints.")
          //
          .def(
            "getCollisionDetectionCPUTimes", &Self::getCollisionDetectionCPUTimes, bp::args("self"),
            "Get timings of the collision detection stage.");

        // Register handles
        {
          // Check registration
          {
            const bp::type_info info = bp::type_id<ModelHandle>();
            const bp::converter::registration * reg = bp::converter::registry::query(info);
            if (!reg)
            {
              bp::register_ptr_to_python<ModelHandle>();
            }
          }
          {
            const bp::type_info info = bp::type_id<DataHandle>();
            const bp::converter::registration * reg = bp::converter::registry::query(info);
            if (!reg)
            {
              bp::register_ptr_to_python<DataHandle>();
            }
          }
          {
            const bp::type_info info = bp::type_id<GeometryModelHandle>();
            const bp::converter::registration * reg = bp::converter::registry::query(info);
            if (!reg)
            {
              bp::register_ptr_to_python<GeometryModelHandle>();
            }
          }
          {
            const bp::type_info info = bp::type_id<GeometryDataHandle>();
            const bp::converter::registration * reg = bp::converter::registry::query(info);
            if (!reg)
            {
              bp::register_ptr_to_python<GeometryDataHandle>();
            }
          }
        }
      }

      static void expose()
      {
        bp::class_<Simulator>("Simulator", "Instance of Simulator.", bp::no_init)
          .def(SimulatorPythonVisitor<Simulator>())
          .def(eigenpy::CopyableVisitor<Simulator>());

        {
          bp::enum_<ConstraintSolverType>("ConstraintSolverType")
            .value("PGS", ConstraintSolverType::PGS)
            .value("ADMM", ConstraintSolverType::ADMM)
            .value("NONE", ConstraintSolverType::NONE);
        }

        {
          bp::class_<ConstraintSolverSettings>(
            "ConstraintSolverSettings", "Settings common to all constraint solvers inside `Simulator`.", bp::no_init)
            .def_readwrite("max_iter", &ConstraintSolverSettings::max_iter, "Max number of iteration of the constraint solver.")
            .def_readwrite(
              "absolute_precision", &ConstraintSolverSettings::absolute_precision,
              "Absolute convergence precision of the constraint solver.")
            .def_readwrite(
              "relative_precision", &ConstraintSolverSettings::relative_precision,
              "Relative convergence precision of the constraint solver.")
            .def_readwrite("stat_record", &ConstraintSolverSettings::stat_record, "Record metrics of the constraint solver.");
        }

        {
          bp::class_<ADMMConstraintSolverSettings, bp::bases<ConstraintSolverSettings>>(
            "ADMMConstraintSolverSettings", "Settings for the ADMM constraint solver inside `Simulator`.", bp::no_init)
            .def_readwrite("mu", &ADMMConstraintSolverSettings::mu, "Proximal parameter of ADMM.")
            .def_readwrite(
              "tau", &ADMMConstraintSolverSettings::tau, "ADMM augmented lagragian penalty is tau * rho (rho is scaled during iterations).")
            .def_readwrite("rho", &ADMMConstraintSolverSettings::rho, "Initial value of rho when using the linear update rule.")
            .def_readwrite(
              "rho_power", &ADMMConstraintSolverSettings::rho_power, "Initial value of rho_power when using the spectral update rule.")
            .def_readwrite("rho_power_factor", &ADMMConstraintSolverSettings::rho_power_factor, "Update factor on rho_power.")
            .def_readwrite(
              "linear_update_rule_factor", &ADMMConstraintSolverSettings::linear_update_rule_factor,
              "Update factor on rho when using linear update rule.")
            .def_readwrite(
              "ratio_primal_dual", &ADMMConstraintSolverSettings::ratio_primal_dual, "Ratio above/below which to trigger the rho update.")
            .def_readwrite(
              "lanczos_size", &ADMMConstraintSolverSettings::lanczos_size,
              "Size of Lanczos decomposition. Higher yields more accurate delassus eigenvalues estimates.")
            .def_readwrite(
              "admm_update_rule", &ADMMConstraintSolverSettings::admm_update_rule,
              "Update rule for the ADMM constraint solver (linear or spectral)");
        }

        {
          bp::class_<PGSConstraintSolverSettings, bp::bases<ConstraintSolverSettings>>(
            "PGSConstraintSolverSettings", "Settings for the PGS constraint solver inside `Simulator`.", bp::no_init)
            .def_readwrite("over_relax", &PGSConstraintSolverSettings::over_relax, "Optional over relaxation value, default to 1.");
        }
      }
    };

  } // namespace python
} // namespace simple

#undef SIMPLE_SIMULATOR_ADD_READONLY_HANDLE

#endif // ifndef __simple_python_algorithm_simulator_hpp__
