use cxx::{memory::UniquePtrTarget, UniquePtr};
// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
use moveit::{CopyNew, New};

use std::{marker::PhantomPinned, mem::MaybeUninit, pin::Pin};

/// A trait which is used to receive any C++ parameter passed by value.
/// This trait is implemented both for references `&T` and for `UniquePtr<T>`,
/// subject to the presence or absence of suitable copy and move constructors.
/// This allows you to pass in parameters by copy (as is ergonomic and normal
/// in C++) retaining the original parameter; or by move semantics thus
/// destroying the object you're passing in. Simply use a reference if you want
/// copy semantics, or the item itself if you want move semantics.
/// It is not recommended that you implement this trait.
pub trait ValueParam<T> {
    /// Any stack storage required. If, as part of passing to C++,
    /// we need to store a temporary copy of the value, this will be `T`,
    /// otherwise `()`.
    type StackStorage;
    fn needs_stack_space(&self) -> bool;
    fn populate_stack_space(&self, this: Pin<&mut MaybeUninit<Self::StackStorage>>);
    /// Return a pointer to the storage.
    fn get_ptr(&mut self) -> *mut T;
}

impl<T> ValueParam<T> for &T
where
    T: CopyNew,
{
    type StackStorage = T;

    fn needs_stack_space(&self) -> bool {
        true
    }

    fn populate_stack_space(&self, this: Pin<&mut MaybeUninit<Self::StackStorage>>) {
        unsafe { crate::moveit::new::copy(*self).new(this) }
    }

    fn get_ptr(&mut self) -> *mut T {
        std::ptr::null_mut()
    }
}

impl<T> ValueParam<T> for UniquePtr<T>
where
    T: UniquePtrTarget,
{
    type StackStorage = ();

    fn needs_stack_space(&self) -> bool {
        false
    }

    fn populate_stack_space(&self, _: Pin<&mut MaybeUninit<Self::StackStorage>>) {}

    fn get_ptr(&mut self) -> *mut T {
        (unsafe {
            Pin::into_inner_unchecked(
                self.as_mut()
                    .expect("Passed a NULL UniquePtr as a C++ value parameter"),
            )
        }) as *mut T
    }
}

/// Implementation detail for how we pass value parameters into C++.
#[doc(hidden)]
pub struct ValueParamHandler<T, VP: ValueParam<T>> {
    param: VP,
    space: Option<MaybeUninit<VP::StackStorage>>,
    _pinned: PhantomPinned,
}

impl<T, VP: ValueParam<T>> ValueParamHandler<T, VP> {
    /// Create a new storage space for something that's about to be passed
    /// by value to C++. Depending on the `ValueParam` type passed in,
    /// this may be largely a no-op or it may involve storing a whole
    /// extra copy of the type.
    ///
    /// # Safety
    ///
    /// Callers must guarantee that this type will not move
    /// in memory.
    pub unsafe fn new(param: VP) -> Self {
        let mut this = Self {
            param,
            space: None,
            _pinned: PhantomPinned,
        };
        if this.param.needs_stack_space() {
            this.space = Some(MaybeUninit::uninit());
            this.param
                .populate_stack_space(Pin::new_unchecked(this.space.as_mut().unwrap()));
        }
        this
    }

    /// Return a pointer to the underlying value which can be passed to C++.
    /// Per the unsafety contract of `new`, the object must not have moved
    /// since it was created.
    pub fn get_ptr(&mut self) -> *mut T {
        if let Some(ref mut space) = self.space {
            let ptr =
                unsafe { space.assume_init_mut() } as *mut <VP as ValueParam<T>>::StackStorage;
            unsafe { std::mem::transmute(ptr) }
        } else {
            self.param.get_ptr()
        }
    }
}

impl<T, VP: ValueParam<T>> Drop for ValueParamHandler<T, VP> {
    fn drop(&mut self) {
        if let Some(space) = self.space.take() {
            unsafe { std::mem::drop(space.assume_init()) };
        }
    }
}
