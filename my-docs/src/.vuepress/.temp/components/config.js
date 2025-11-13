import { hasGlobalComponent } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+helper@2.0.0-rc.1_cff9eb5b88fb6a429afb371eb704a643/node_modules/@vuepress/helper/lib/client/index.js";
import Badge from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-plugin-components@_3f456054f390d13ef43df9b3e8228b04/node_modules/vuepress-plugin-components/lib/client/components/Badge.js";
import VPCard from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-plugin-components@_3f456054f390d13ef43df9b3e8228b04/node_modules/vuepress-plugin-components/lib/client/components/VPCard.js";

import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+helper@2.0.0-rc.1_cff9eb5b88fb6a429afb371eb704a643/node_modules/@vuepress/helper/lib/client/styles/sr-only.css";

export default {
  enhance: ({ app }) => {
    if(!hasGlobalComponent("Badge")) app.component("Badge", Badge);
    if(!hasGlobalComponent("VPCard")) app.component("VPCard", VPCard);
    
  },
  setup: () => {

  },
  rootComponents: [

  ],
};
