#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to fake_tag_interfaces__msg__TagDetection

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TagDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub tag_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::Pose,

}



impl Default for TagDetection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TagDetection::default())
  }
}

impl rosidl_runtime_rs::Message for TagDetection {
  type RmwMsg = super::msg::rmw::TagDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        tag_id: msg.tag_id,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      tag_id: msg.tag_id,
        pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      tag_id: msg.tag_id,
      pose: geometry_msgs::msg::Pose::from_rmw_message(msg.pose),
    }
  }
}


// Corresponds to fake_tag_interfaces__msg__TagDetectionArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TagDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: Vec<super::msg::TagDetection>,

}



impl Default for TagDetectionArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TagDetectionArray::default())
  }
}

impl rosidl_runtime_rs::Message for TagDetectionArray {
  type RmwMsg = super::msg::rmw::TagDetectionArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::TagDetection::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::TagDetection::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::TagDetection::from_rmw_message)
          .collect(),
    }
  }
}


