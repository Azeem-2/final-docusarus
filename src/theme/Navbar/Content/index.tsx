import React from "react";
import NavbarContent from "@theme-original/Navbar/Content";
import ProfileMenu from "@site/src/components/ProfileMenu";

export default function NavbarContentWrapper(props: any) {
  return (
    <>
      <NavbarContent {...props} />
      <ProfileMenu />
    </>
  );
}


