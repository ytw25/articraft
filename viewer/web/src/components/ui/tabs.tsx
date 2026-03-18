import * as React from "react";
import * as TabsPrimitive from "@radix-ui/react-tabs";

import { cn } from "@/lib/utils";

export function Tabs({
  className,
  ...props
}: React.ComponentProps<typeof TabsPrimitive.Root>): React.JSX.Element {
  return <TabsPrimitive.Root className={cn("flex flex-col gap-0", className)} {...props} />;
}

export function TabsList({
  className,
  ...props
}: React.ComponentProps<typeof TabsPrimitive.List>): React.JSX.Element {
  return (
    <TabsPrimitive.List
      className={cn("inline-flex h-auto items-center bg-transparent p-0 text-[#999]", className)}
      {...props}
    />
  );
}

export function TabsTrigger({
  className,
  ...props
}: React.ComponentProps<typeof TabsPrimitive.Trigger>): React.JSX.Element {
  return (
    <TabsPrimitive.Trigger
      className={cn(
        "inline-flex items-center justify-center px-2.5 py-2 text-[11px] font-medium transition-colors data-[state=active]:text-[#1e1e1e] focus-visible:outline-1 focus-visible:outline-offset-1 focus-visible:outline-[#007acc]",
        className,
      )}
      {...props}
    />
  );
}

export function TabsContent({
  className,
  ...props
}: React.ComponentProps<typeof TabsPrimitive.Content>): React.JSX.Element {
  return <TabsPrimitive.Content className={cn("outline-none", className)} {...props} />;
}
