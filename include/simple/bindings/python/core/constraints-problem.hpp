//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_python_core_constraints_problem_hpp__
#define __simple_python_core_constraints_problem_hpp__

#include "simple/core/constraints-problem.hpp"
#include "simple/bindings/python/fwd.hpp"

#include <eigenpy/std-vector.hpp>
#include <eigenpy/copyable.hpp>

#define SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(type, name, desc)                                                                  \
  add_property(                                                                                                                            \
    #name, bp::make_function(+[](Self & self) -> type { return self.name(); }),                                                            \
    bp::make_function(+[](Self &, const type &) -> void { PINOCCHIO_THROW(std::invalid_argument, "Cannot manually set " #name ".") }),     \
    desc)

namespace simple
{
  namespace python
  {
    namespace bp = boost::python;

    // template<typename T>
    // struct StdVectorOfReferenceWrappersPythonVisitor : public bp::def_visitor<StdVectorOfReferenceWrappersPythonVisitor<T>>
    // {
    //   typedef std::vector<std::reference_wrapper<T>> Self;
    //
    // public:
    //   template<class PyClass>
    //   void visit(PyClass & cl) const
    //   {
    //     cl.def(
    //         "__getitem__", +[](const Self & self, std::size_t i) -> const T & { return self[i].get(); },
    //         bp::return_value_policy<bp::reference_existing_object>())
    //       .def("__len__", +[](const Self & self) -> std::size_t { return self.size(); });
    //   }
    //
    //   static void expose(const std::string & name)
    //   {
    //     bp::class_<Self>(name.c_str(), "Vector of reference wrappers.", bp::no_init).def(StdVectorOfReferenceWrappersPythonVisitor());
    //   }
    // };

    template<typename ConstraintsProblem>
    struct ConstraintsProblemPythonVisitor : public bp::def_visitor<ConstraintsProblemPythonVisitor<ConstraintsProblem>>
    {
      typedef typename ConstraintsProblem::Scalar Scalar;
      typedef ConstraintsProblem Self;

      using ModelHandle = typename Self::ModelHandle;
      using DataHandle = typename Self::DataHandle;
      using VectorXs = typename Self::VectorXs;
      using GeometryModelHandle = typename Self::GeometryModelHandle;
      using GeometryDataHandle = typename Self::GeometryDataHandle;
      using PlacementVector = typename Self::PlacementVector;
      using ConstraintModel = typename Self::ConstraintModel;
      using ConstraintData = typename Self::ConstraintData;
      using BilateralPointConstraintModelVector = typename Self::BilateralPointConstraintModelVector;
      using WeldConstraintModelVector = typename Self::WeldConstraintModelVector;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
          // ----------------------------------
          // CONSTRUCTORS
          .def(bp::init<ModelHandle, DataHandle, GeometryModelHandle, GeometryDataHandle>(
            bp::args("self", "model", "data", "geom_model", "geom_data"), "Constructor"))

          // ----------------------------------
          // ATTRIBUTES/METHODS

          // ----------------------------------
          // -- general
          .def_readonly(
            "constraint_cholesky_decomposition", &Self::constraint_cholesky_decomposition,
            "Cholesky decomposition of the constraints problem. In particular, it contains the "
            "Cholesky decomposition of the Delassus' operator `G`.")
          .def_readwrite(
            "is_ncp", &Self::is_ncp, "Type of constraints problem. If set to true, the constraints problem is a NCP, else it is a CCP.")

          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(int, constraints_problem_size, "Size of the full constraint problem vector.")

          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(VectorXs, g, "Drift term `g` of the constraints problem.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(VectorXs, preconditioner, "Preconditioner of the constraints problem.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, constraints_forces, "Constraint forces of the current contact problem.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_constraints_forces, "Constraint forces of the previous contact problem.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, constraints_velocities, "Constraint velocities divided by dt (v_c / dt).")

          .def("update", &Self::update, bp::args("self"), "Update constraints with current model, data, geom_model, geom_data.")

          .def("clear", &Self::clear, bp::args("self"), "Clear currrent contact quantities.")

          // ----------------------------------
          // -- joint friction
          .def_readwrite("joint_friction_constraint_models", &Self::joint_friction_constraint_model, "Joint friction constraint model.")
          .def_readonly("joint_friction_constraint_datas", &Self::joint_friction_constraint_data, "Joint friction constraint data.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            int, joint_friction_constraint_size, "Size of the vector of dry friction force vector.")
          .def(
            "getNumberOfJointFrictionConstraints", &Self::getNumberOfJointFrictionConstraints, bp::args("self"),
            "Returns the number of joint friction constraints.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, joint_friction_constraint_forces, "Joint friction constraints' forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_joint_friction_constraint_forces, "Joint friction constraints' forces at the previous time step.")

          // ----------------------------------
          // -- bilateral constraints
          .def_readwrite(
            "bilateral_point_constraint_models", &Self::bilateral_point_constraint_models, "Vector of bilateral constraint models.")
          .def_readonly(
            "bilateral_point_constraint_datas", &Self::bilateral_point_constraint_datas, "Vector of bilateral constraint datas.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            int, bilateral_constraints_size, "Size of the vector of bilateral constraint force vector.")
          .def(
            "getNumberOfBilateralConstraints", &Self::getNumberOfBilateralConstraints, bp::args("self"),
            "Returns the number of bilateral constraints.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(VectorXs, bilateral_constraints_forces, "Bilateral constraints' forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_bilateral_constraints_forces, "Bilateral constraints' forces at the previous time step.")

          // ----------------------------------
          // -- weld constraints
          .def_readwrite("weld_constraint_models", &Self::weld_constraint_models, "Vector of weld constraint models.")
          .def_readonly("weld_constraint_models", &Self::weld_constraint_models, "Vector of weld constraint datas.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            int, weld_constraints_size, "Size of the vector of weld constraint force vector.")
          .def("getNumberOfWeldConstraints", &Self::getNumberOfWeldConstraints, bp::args("self"), "Returns the number of weld constraints.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(VectorXs, weld_constraints_forces, "Weld constraints' forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_weld_constraints_forces, "Weld constraints' forces at the previous time step.")

          // ----------------------------------
          // -- joint limits
          .def_readwrite("joint_limit_constraint_model", &Self::joint_limit_constraint_model, "Joint limit constraint model.")
          .def_readonly("joint_limit_constraint_data", &Self::joint_limit_constraint_data, "Joint limit constraint data.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(int, joint_limit_constraint_size, "Size of joint limit forces vector.")
          .def(
            "getNumberOfJointLimitConstraints", &Self::getNumberOfJointLimitConstraints, bp::args("self"),
            "Returns the number of joint limit constraints.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(VectorXs, joint_limit_constraint_forces, "Joint limit constraint's forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_joint_limit_constraint_forces, "Joint limit constraint's forces at the previous time step.")

          // ----------------------------------
          // -- frictional point constraints
          .def_readwrite(
            "frictional_point_constraint_models", &Self::frictional_point_constraint_models,
            "Vector of frictional point constraint models.")
          .def_readonly(
            "frictional_point_constraint_datas", &Self::frictional_point_constraint_datas, "Vector of frictional point constraint datas.")
          .def(
            "setMaxNumberOfContactsPerCollisionPair", &Self::setMaxNumberOfContactsPerCollisionPair, bp::args("self"),
            "Set maximum number of contacts for each collision pair.")
          .def(
            "getMaxNumberOfContactsPerCollisionPair", &Self::getMaxNumberOfContactsPerCollisionPair, bp::args("self"),
            "Get the maximum number of contacts for each collision pair.")
          .def(
            "getMaxNumberOfContacts", &Self::getMaxNumberOfContacts, bp::args("self"),
            "Maximum number of contacts this `ConstraintsProblem` can handle.")

          .def("getNumberOfContacts", &Self::getNumberOfContacts, bp::args("self"), "Number of contacts.")
          .def("getPreviousNumberOfContacts", &Self::getPreviousNumberOfContacts, bp::args("self"), "Previous number of contacts.")

          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            int, frictional_point_constraints_size, "Size of the vector of the frictional point constraints' forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, frictional_point_constraints_forces, "Frictional point constraints' forces.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, previous_frictional_point_constraints_forces, "Frictional point constraints' forces at the previous time step.")

          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            VectorXs, frictional_point_constraints_velocities, "Frictional point constraints' velocities scaled by dt (v_c * dt).")

          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            PlacementVector, frictional_point_constraints_placements, "Contact placements (oMc) of the current contact problem.")
          .SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY(
            PlacementVector, previous_frictional_point_constraints_placements,
            "Contact placements (oMc) at the previous time step (previous contact problem).")

          .def_readonly("pairs_in_collision", &Self::pairs_in_collision, "Ids of collision pairs which are in collision.")
          .def_readonly(
            "contact_id_to_collision_pair", &Self::contact_id_to_collision_pair,
            "Vector that maps the id of the contact to the collision pair. Therefore "
            "`contact_id_to_collision_pair[i]` is the id of the collision pair corresponding to "
            "the i-th contact. Note: since collision pairs can have multiple contacts, the same id "
            "can be found multiple times inside `contact_id_to_collision_pair`.")
          .def_readonly("contact_mappers", &Self::contact_mappers, "Vector of contact mappers of the current contact problem.")

          .def_readonly(
            "contact_modes", &Self::contact_modes,
            "Contact modes associated to frictional point constraints (breadking, sliding or sticking).")
          .def(
            "collectActiveSet", &Self::collectActiveSet, (bp::arg("self"), bp::arg("epsilon") = 1e-6),
            "Collect active set of of the solution of the contact problem.");

        // StdVectorOfReferenceWrappersPythonVisitor<const ConstraintModel>::expose("StdRefVec_ConstraintModel");
        // StdVectorOfReferenceWrappersPythonVisitor<ConstraintData>::expose("StdRefVec_ConstraintData");

        cl.def_readonly("constraint_models", &Self::constraint_models, "Active constraint models.")
          .def_readonly("constraint_datas", &Self::constraint_datas, "Active constraint datas.");
      }

      static void expose()
      {
        eigenpy::StdVectorPythonVisitor<typename ConstraintsProblem::BilateralPointConstraintModelVector, true>::expose(
          "StdVec_BilateralPointConstraintModel");

        eigenpy::StdVectorPythonVisitor<typename ConstraintsProblem::WeldConstraintModelVector, true>::expose(
          "StdVec_WeldConstraintModel");

        eigenpy::StdVectorPythonVisitor<std::vector<double>, true>::expose("StdVec_double");

        eigenpy::StdVectorPythonVisitor<std::vector<typename ConstraintsProblem::ContactMapper>, true>::expose(
          "StdVec_ContactMapper");

        eigenpy::StdVectorPythonVisitor<std::vector<typename ConstraintsProblem::ContactMode>, true>::expose(
          "StdVec_ContactMode");

        bp::class_<typename ConstraintsProblem::WrappedConstraintModel>("WrappedConstraintModel", bp::no_init)
          .def(
            "__getattr__",
            +[](bp::object self, std::string const & name) {
              using ConstraintModel = typename ConstraintsProblem::ConstraintModel;
              const ConstraintModel & obj = bp::extract<ConstraintModel>(self());
              return bp::getattr(bp::object(bp::ptr(&obj)), name.c_str());
            })
          .def(
            "__call__",
            +[](const typename ConstraintsProblem::WrappedConstraintModel & cmodel)
              -> const typename ConstraintsProblem::ConstraintModel & { return cmodel.get(); },
            bp::return_internal_reference<>());

        bp::class_<typename ConstraintsProblem::WrappedConstraintData>("WrappedConstraintData", bp::no_init)
          .def(
            "__getattr__",
            +[](bp::object self, std::string const & name) {
              using ConstraintData = typename ConstraintsProblem::ConstraintData;
              const ConstraintData & obj = bp::extract<ConstraintData>(self());
              return bp::getattr(bp::object(bp::ptr(&obj)), name.c_str());
            })
          .def(
            "__call__",
            +[](const typename ConstraintsProblem::WrappedConstraintData & cdata) -> const typename ConstraintsProblem::ConstraintData & {
              return cdata.get();
            },
            bp::return_internal_reference<>());

        eigenpy::StdVectorPythonVisitor<typename ConstraintsProblem::WrappedConstraintModelVector, true>::expose(
          "StdVec_WrappedConstraintModel");
        eigenpy::StdVectorPythonVisitor<typename ConstraintsProblem::WrappedConstraintDataVector, true>::expose(
          "StdVec_WrappedConstraintData");

        bp::class_<ConstraintsProblem>("ConstraintsProblem", "Contact problem.\n", bp::no_init)
          .def(ConstraintsProblemPythonVisitor<ConstraintsProblem>())
          .def(eigenpy::CopyableVisitor<ConstraintsProblem>());
      }
    };

  } // namespace python
} // namespace simple

#undef SIMPLE_CONSTRAINT_PROBLEM_ADD_READONLY_PROPERTY

#endif // ifndef__simple_python_core_constraints_problem_hpp__
