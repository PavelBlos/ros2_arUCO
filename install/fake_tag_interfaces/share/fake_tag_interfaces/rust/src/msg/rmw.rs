#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "fake_tag_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fake_tag_interfaces__msg__TagDetection() -> *const std::ffi::c_void;
}

#[link(name = "fake_tag_interfaces__rosidl_generator_c")]
extern "C" {
    fn fake_tag_interfaces__msg__TagDetection__init(msg: *mut TagDetection) -> bool;
    fn fake_tag_interfaces__msg__TagDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TagDetection>, size: usize) -> bool;
    fn fake_tag_interfaces__msg__TagDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TagDetection>);
    fn fake_tag_interfaces__msg__TagDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TagDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<TagDetection>) -> bool;
}

// Corresponds to fake_tag_interfaces__msg__TagDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TagDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub tag_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::Pose,

}



impl Default for TagDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fake_tag_interfaces__msg__TagDetection__init(&mut msg as *mut _) {
        panic!("Call to fake_tag_interfaces__msg__TagDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TagDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TagDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TagDetection where Self: Sized {
  const TYPE_NAME: &'static str = "fake_tag_interfaces/msg/TagDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fake_tag_interfaces__msg__TagDetection() }
  }
}


#[link(name = "fake_tag_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fake_tag_interfaces__msg__TagDetectionArray() -> *const std::ffi::c_void;
}

#[link(name = "fake_tag_interfaces__rosidl_generator_c")]
extern "C" {
    fn fake_tag_interfaces__msg__TagDetectionArray__init(msg: *mut TagDetectionArray) -> bool;
    fn fake_tag_interfaces__msg__TagDetectionArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TagDetectionArray>, size: usize) -> bool;
    fn fake_tag_interfaces__msg__TagDetectionArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TagDetectionArray>);
    fn fake_tag_interfaces__msg__TagDetectionArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TagDetectionArray>, out_seq: *mut rosidl_runtime_rs::Sequence<TagDetectionArray>) -> bool;
}

// Corresponds to fake_tag_interfaces__msg__TagDetectionArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TagDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::TagDetection>,

}



impl Default for TagDetectionArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fake_tag_interfaces__msg__TagDetectionArray__init(&mut msg as *mut _) {
        panic!("Call to fake_tag_interfaces__msg__TagDetectionArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TagDetectionArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetectionArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetectionArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fake_tag_interfaces__msg__TagDetectionArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TagDetectionArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TagDetectionArray where Self: Sized {
  const TYPE_NAME: &'static str = "fake_tag_interfaces/msg/TagDetectionArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fake_tag_interfaces__msg__TagDetectionArray() }
  }
}


