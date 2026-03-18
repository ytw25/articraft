import * as React from "react";
import { cva, type VariantProps } from "class-variance-authority";

import { cn } from "@/lib/utils";

const badgeVariants = cva("inline-flex max-w-full items-center rounded-sm px-1.5 py-0.5 font-mono text-[10px] font-medium", {
  variants: {
    variant: {
      default: "bg-[#007acc]/10 text-[#007acc]",
      success: "bg-[#16a34a]/10 text-[#15803d]",
      warning: "bg-[#d97706]/10 text-[#b45309]",
      destructive: "bg-[#dc2626]/10 text-[#dc2626]",
      secondary: "bg-[#f0f0f0] text-[#888]",
    },
  },
  defaultVariants: {
    variant: "default",
  },
});

export type BadgeProps = React.HTMLAttributes<HTMLSpanElement> & VariantProps<typeof badgeVariants>;

export function Badge({ className, variant, ...props }: BadgeProps): React.JSX.Element {
  return <span className={cn(badgeVariants({ variant, className }))} {...props} />;
}
