# VLA 在机械臂操作中的持续学习与长时序：单臂/双臂发展脉络、代表论文与研究地图

## 执行摘要

这条研究线索可以概括为“三股力量渐进汇流，但四重交叉仍显著空白”：其一，**语言条件操作（language-conditioned manipulation）**在 2019–2022 年间通过 RLBench、CLIPort、BC-Z、PerAct、CALVIN 等基准与方法把“语言→操作”问题工程化、可评测化；其二，2022–2024 年形成以 RT-1/RT-2、Open X-Embodiment/RT-X、Octo、OpenVLA 为代表的“**通用/基础策略（generalist/foundation policy）**”浪潮，把多任务、多机构、多 embodiment 数据与大模型范式带入真实机器人操作；其三，**终身/持续学习（lifelong/continual learning）**在操作领域以 LIBERO 为核心枢纽形成较独立的评测与方法谱系，并在 2025–2026 通过参数高效适配与 RL 后训练开始与 VLA 主线发生强连接。citeturn18search0turn22search0turn8search1turn11search1turn13search2turn16search20turn16search18turn16search8turn13search15turn20search0turn35search1turn15search0

关键结论（直接回答你最关心的“判断题”）如下。

第一，**机械臂操作场景下 VLA 的发展主线**已经相对清晰：从“语言作为条件/奖励/目标描述”的多任务 BC/IL（CLIPort、BC-Z、PerAct）与长时序组合评测（CALVIN）出发，走向在大规模真实示教与跨 embodiment 数据上预训练的通用策略（RT-1/RT-2、RT-X、Octo、OpenVLA、π₀/π₀.₅），再进一步用扩散/flow matching/离散生成式 action expert 提升连续控制与鲁棒性，并通过记忆/检索/层级结构补齐长时序依赖。citeturn22search0turn8search1turn11search1turn13search1turn16search20turn16search6turn16search18turn16search8turn13search15turn20search0turn20search1turn24search5turn28view0

第二，**“VLA + 持续学习”在 2025–2026 快速成形，但尚谈不上成熟路线**：CLARE、CRL-VLA、Simple Recipe Works 等工作把“参数高效适配（LoRA/adapter）+ 顺序训练/持续 RL 后训练”作为核心抓手，报告了在 LIBERO 等终身操作基准上显著减轻遗忘、维持可塑性与可扩展性的结果；但它们大多仍集中在**单臂/桌面**、**任务边界相对清晰**、以及**基准内分布**设定上，距离“开放世界长期在线运行”的严格内涵仍有距离。citeturn35search1turn15search12turn15search0turn13search2turn35search6turn6search5turn37search3

第三，**“VLA + 长时序操作”有两条路径并行推进且正在靠拢**：一条是以 CALVIN 为代表的“长时序语言条件策略”路径（HULC/RoboFlamingo 等），通过层级表示、历史建模与更合适的数据组织提升平均成功链长；另一条是以 Long-VLA、MemoryVLA、MemER、MMaDA-VLA、MGP-Long 为代表的“在 VLA 框架内显式建模长时序”路径（phase-aware、记忆库/检索、离散/扩散式全局一致性生成）。这两条线在 2025–2026 开始出现技术互借，但仍未完全统一为一个主流范式。citeturn13search1turn33search7turn33search11turn27view0turn32view0turn28view0turn6search11turn15search1

第四，**“VLA + 双臂 + 长时序 + 持续学习”的四重交叉点仍然非常空白**：双臂方向虽然出现了 ALOHA/移动 ALOHA、PerAct2、RoboTwin、BiGym、AnyBimanual 等数据/基准/迁移框架，但这些工作多数聚焦“协同控制/任务多样性/单次训练泛化/从单臂迁移到双臂”，很少采用 LIBERO 式持续评测协议，更少把“分钟级历史依赖”与“顺序增量任务流”同时纳入。换言之，这个交叉点仍是一个清晰的研究机会区，而不是已被充分覆盖的热点末端。citeturn12search9turn9search2turn11search3turn10search0turn12search12turn12search3turn13search2turn28view0turn32view0

**研究判断（critical takeaways）**：现阶段最可靠的“工程可落地路线”是“通用 VLA 预训练（跨任务/跨 embodiment）→ 参数高效适配（LoRA/adapter）→（可选）RL/离线 RL 后训练增强鲁棒性”，而长时序与双臂能力的系统性提升仍更依赖：更严格的评测协议、更可扩展的数据采集（尤其双臂/接触丰富/多视角）、以及把“记忆/技能库/层级结构”与“持续适配”统一到同一训练与部署闭环中。citeturn13search15turn20search0turn20search1turn16search18turn35search1turn15search0turn28view0turn32view0turn11search3turn10search0

## 术语澄清与范围界定

为避免把“VLM + 规划”与“VLA + 操作策略”混为一谈，本节明确**术语边界**与文献中常见口径差异，并将讨论严格锚定在**机械臂/双臂 manipulation**。

**VLA 与相关概念**

VLA（Vision-Language-Action）在机器人操作语境中通常指：**输入包含视觉观测与语言指令（常可加状态/力觉/多视角），输出直接是动作（连续控制或离散动作 tokens）**的策略模型。与纯 VLM 不同，VLA 的训练目标不是“生成文字”，而是“生成可执行控制信号”。但社区对“是否必须端到端、是否允许层级拆分”并无绝对一致口径：一些综述把“直接生成控制命令”作为狭义定义，另一些则把包含高层决策与低层控制的系统也纳入广义 VLA。citeturn12search16turn6search20turn16search17turn20search0turn13search20

VLM（vision-language model）强调“视觉-语言对齐与推理”，常输出文本或视觉-语义表征；很多 VLA 直接复用预训练 VLM 作为骨干，再加动作头（action expert）。典型例子是 RT-2 把互联网规模的视觉-语言知识带入控制，或 π₀ 在 VLM 骨干上用 flow matching 输出连续动作。citeturn16search17turn20search0turn34search15

“policy model / robot foundation model / generalist policy”在操作文献里通常指：**在大量多任务示教（可能跨 robot）上预训练形成可迁移的基础策略**。它不一定要求语言输入（也可用 goal image），但 VLA 往往是其语言条件版本。Octo、OpenVLA、RT-X、π₀/π₀.₅、Open X-Embodiment 等都在“跨任务/跨 embodiment 扩展”层面强调 foundation 的意义。citeturn16search8turn13search15turn16search18turn20search0turn20search1

planner–executor pipeline（规划-执行流水线）与 VLA 的核心区别在于：前者通常让 LLM/VLM产生**符号计划、子目标、代码或轨迹约束**，再由现成的低层控制器/技能库/运动规划器执行；而 VLA 更强调“动作生成”本身是学习到的策略输出。SayCan 与 VoxPoser 具有强启发性（它们解决了开放词汇任务分解与可行性过滤），但严格说通常不被视为“单体端到端 VLA”，更适合作为“邻近桥梁文献”。citeturn13search20turn21search12turn28view0turn32view0

diffusion policy / flow matching policy 不是语言概念，而是动作建模范式：扩散/flow matching 用“迭代去噪/流匹配”在连续动作空间建模多模态分布，常用于接触丰富、需要平滑轨迹的操作。近两年大量 VLA 把扩散/flow matching 作为 action expert（例如 Octo、π₀/π₀.₅、MemoryVLA、MMaDA-VLA、MGP-Long），与早期“离散动作 token + 自回归输出”的 VLA（RT-2、OpenVLA 等）形成两大路线。citeturn16search4turn20search0turn20search1turn30view0turn6search11turn15search1

world model / skill library / hierarchical policy 是“长时序”常见的三类解法：world model 通过预测来规划；skill library 把可复用技能当 primitive；hierarchical policy 则显式拆分高层与低层。它们可以与 VLA 组合，但并不等同于 VLA；在操作领域，长期以来“层级 + 技能组合”几乎是解决长时序失败累积的标准工具链，因此在本调研中必须与 VLA 主线并置讨论。citeturn13search1turn17search17turn17search10turn28view0turn32view0

**持续学习相关概念**

continual learning / lifelong learning / incremental learning 在一般机器学习中常以“任务序列 + 遗忘/迁移指标”定义；但在机器人操作里容易与“多任务学习”“顺序微调”“在线适应”混淆。LIBERO 明确把机器人终身学习定位为 LLDM（lifelong learning in decision-making），强调除了视觉-语言的“陈述性知识（declarative）”，还必须迁移“过程性知识（procedural）”即动作与行为；并强调任务顺序影响与预训练影响。citeturn13search2turn13search6turn22search2

online adaptation 是部署时持续与环境交互并更新（可能含 RL），与仅在离线任务流上顺序训练不同。CRL-VLA 与 Simple Recipe Works 明确把“持续 RL 后训练/持续强化学习”作为 VLA 终身学习路径，并讨论稳定性-可塑性权衡。citeturn15search12turn15search0turn36search9

sequential fine-tuning（顺序微调）在传统 CL 里通常会导致灾难性遗忘；但在“大模型 + 参数高效适配 +（可选）RL 后训练”的设置下，近 2025–2026 的一批工作报告“简单顺序训练 surprisingly strong”，这构成了当前 CL+VLA 最重要的争议点之一：到底是“问题被解决了”，还是“评测协议不够严格/任务分布太近/数据泄漏式共性太强”。citeturn15search0turn35search1turn35search6turn6search5

parameter-efficient adaptation（LoRA/adapter/prompt）在机器人操作里既可以被当作“效率技巧”，也可以被当作“持续学习机制”的核心部件：因为它天然实现“旧参数冻结 + 新参数增量”，容易做模块化路由与参数合并（如 CLARE 的 adapter routing/expansion、DMPEL 的低秩 expert 库与系数回放、VLM2VLA 的 LoRA 对齐训练、RETAIN 的参数合并）。citeturn35search1turn35search6turn6search5turn37search3

**长时序相关概念**

long-horizon manipulation 在操作文献中常见三类界定方式：按“子任务链长度/成功链长”、按“时间跨度（秒到分钟）”、按“是否存在跨阶段依赖与误差累积”。CALVIN 的评测以“连续执行多条语言指令链”并用 average length/多步成功率体现长时序组合能力；MemoryVLA 与 MemER 则更接近“分钟级记忆依赖与非马尔可夫性”口径。citeturn13search1turn33search8turn30view0turn28view0

multi-stage manipulation / skill chaining / hierarchical planning 在很多论文中被当作长时序的工程化同义词，但需要警惕：多阶段不等于真正的长期依赖——如果环境在每阶段重置、或阶段之间信息完全可从当前观测恢复，那么它并不要求“记忆”。因此本调研会把“需要历史才能判别状态/进度”的任务单列为“强长时序”。citeturn30view0turn28view0turn17search9

progress estimation（进度估计）与 memory-based policy（记忆策略）是长时序成功率的关键瓶颈：一旦策略无法判断子目标是否达成、或无法在失败重试后对齐到正确阶段，就会发生“阶段漂移”。Skill chaining、检索记忆（MemER）、双记忆系统（MemoryVLA）都是围绕这个痛点的不同取向。citeturn17search10turn28view0turn30view0turn32view0

**操作形态相关概念**

single-arm（unimanual）操作通常指单机械臂完成抓取、放置、开关、插拔等；bimanual（双臂）则要求两个末端执行器在空间与时间上协同，并常带来“内部力、对偶约束、同步接触”问题。PerAct2 直接把“空间与时间协同”作为双臂困难本质并据此构建基准；RoboTwin、ALOHA/移动 ALOHA 等强调双臂数据采集与评测的缺口。citeturn11search3turn10search0turn12search9turn9search2

whole-body manipulation 与 mobile manipulation 在严格意义上不等同于“机械臂操作”，但当移动底盘或身体自由度成为“拿到/搬运/稳定接触”的必要条件时，它们就与双臂长时序高度相关。移动 ALOHA 把双臂与低成本全身遥操作结合，用于更贴近日常场景的移动操作任务；这类工作对“长时序与真实部署”很有启发，应视为邻近重要文献。citeturn9search2turn12search12turn9search3

**研究判断（critical takeaways）**：术语最容易混淆之处在于把“VLM 规划 + 传统控制”当成 VLA，把“多任务训练”当成持续学习，把“多步任务”当成长时序。要做深度研究与选题，建议明确三条红线：是否直接输出控制动作、是否在任务流上评测遗忘/迁移、是否存在必须依赖历史的状态信息。citeturn12search16turn13search2turn13search1turn30view0turn28view0

## 时间线与发展脉络

为了突出“操作 manipulation”主线，下述时间线把 VLA、长时序、终身学习、双臂数据与基准四条线并行叙述，并标注关键转折点。

**早期铺垫：可扩展的操作基准与数据共享（2019–2021）**

2019 年的 RLBench 提供了 100 个手工设计任务与可扩展的演示生成机制，使“多任务、可复现、可做 few-shot/multi-stage”的操作评测成为可能；同年 RoboNet 以多机器人视频数据强调“跨平台共享经验”的价值。citeturn18search0turn38search2

这一阶段的重点不是 VLA，而是把操作学习问题变成“可规模化数据 + 标准化任务套件”，为后续语言条件与大模型化奠基。citeturn18search0turn38search2

**语言条件操作成为主流问题设定（2021–2022）**

CLIPort 用“what/where”路径把开放词汇语义与空间精度结合，在少量数据下实现多任务语言条件操作，为后续“语言作为通用接口”奠定典型范式。citeturn22search0turn22search4

CALVIN 明确提出“长时序语言条件操作基准”，以连续语言指令链与成功链长衡量组合能力，并显示早期基线（MCIL）仍远未解决长时序组合问题。citeturn13search1turn33search9turn25search3

BC-Z 探索通过扩大与多样化示教数据实现“新任务零样本泛化”，将“规模化示教 + 任务嵌入”推到台前。citeturn8search1turn8search18

**通用机器人策略与 VLA 兴起（2022–2024）**

RT-1 把 Transformer 端到端用于真实机器人多任务控制，强调用大规模多任务轨迹吸收多样性；RT-2 在此基础上把互联网规模视觉-语言知识引入控制，成为“VLM→VLA”叙事的标志性转折点。citeturn16search20turn16search9turn16search17turn16search6

Open X-Embodiment/RT-X 把多个机构的真实机器人数据统一格式并发布跨 22 种机器人 embodiment 规模数据，使“跨 embodiment 预训练”成为开放研究主题。citeturn16search18turn16search7turn16search10

与此同步，BridgeData V2、DROID 等真实机器人数据集把“多环境、多站点、多任务”的真实数据规模推高，推动了“可公开复现的真实世界泛化”研究。citeturn10search3turn10search11turn38search0turn38search1turn38search9

OpenVLA 在开源层面具有里程碑意义：提供 7B 规模开源 VLA，并明确强调多机器人控制与参数高效微调的可用性。citeturn13search15turn13search7turn13search3

**持续学习在操作中的独立发展与回流（2023–2026）**

LIBERO 把“终身机器人操作”明确化为 LLDM，提供任务套件与指标，并报告一个对社区影响深远的现象：在其设定下，简单顺序微调在前向迁移上可能优于一些复杂 CL 方法，从而引发“评测/设定/先验”层面的再思考。citeturn13search2turn13search6turn13search10

随后出现两类值得关注的回流：其一是“技能库/原语”路线（LOTUS、PPL）把持续学习解释为“不断发现与复用技能 primitive”；其二是“参数模块化/PEFT”路线（DMPEL、CLARE、VLM2VLA、RETAIN）把持续学习解释为“冻结基础能力 + 增量参数 + 路由/合并/对齐”。citeturn19search2turn19search14turn35search6turn35search1turn6search5turn37search3

2026 年进一步出现“持续 RL 后训练”叙事：CRL-VLA 给出持续 RL 的理论视角与结构化解法，而 Simple Recipe Works 则主张“预训练 VLA + LoRA + 顺序 RL 微调”已经能显著缓解遗忘并保持泛化。citeturn15search12turn15search0turn36search9

**双臂操作数据集/平台与策略学习（2023–2025）**

ALOHA 用低成本双臂硬件与遥操作数据证明“高质量双臂示教 + 合适的序列建模（如 ACT）”可以高效学习精细双臂技能；移动 ALOHA 把这一路线扩展到双臂移动操作与全身遥操作。citeturn12search9turn9search2

PerAct2 以“扩展 RLBench 到双臂”补齐双臂多任务多变体基准空缺；RoboTwin 通过生成式数字孪生强调双臂任务可扩展数据生成与统一评测；BiGym 则面向移动双臂 demo-driven 场景。citeturn11search3turn10search0turn12search12turn10search8

AnyBimanual 把“从单臂通用策略迁移到双臂”作为降低数据成本的核心问题，体现出“双臂数据贵、而单臂基础能力可复用”的重要研究洞见。citeturn12search3turn12search7turn12search11

**最近两年融合趋势：VLA × 长时序 ×（少量）持续机制（2025–2026）**

长时序方面，Long-VLA 提出 phase-aware masking 并引入 L-CALVIN 评测；MemoryVLA 与 MemER 则更明确地把“长期记忆/检索”作为 VLA 的关键部件；MMaDA-VLA、MGP-Long 等生成式策略尝试从“全局一致性生成/迭代 refine”角度压制长时序误差累积。citeturn27view0turn32view0turn28view0turn6search11turn15search1turn15search5

持续学习方面，CLARE、CRL-VLA、Simple Recipe Works 等把“VLA 的持续更新”问题前置为核心议题，并以 LIBERO 作为主要验证场景；同时出现更严格的鲁棒评测诉求，如 LIBERO-X 针对多维分布偏移下的真实性能退化提出层级评测。citeturn35search1turn15search12turn15search0turn13search2turn37search18

**研究判断（critical takeaways）**：大趋势已经从“单任务/小数据/单平台”转向“跨任务/跨平台预训练 + 低成本适配”，但四个难点（双臂协同、长时序依赖、持续更新、真实世界鲁棒）仍大多被分开解决。未来最有价值的工作往往不是再做一个更大的 VLA，而是提出统一评测与统一机制，把这些难点放到同一个闭环里对齐。citeturn16search18turn20search0turn11search3turn28view0turn35search1turn37search18

## 分类框架

为了形成“选题地图”，这里给出一个可操作的**三维主框架 + 六类机制轴**，用来把文献放到同一坐标系里比较，而不是按年份堆叠。

**主框架三维坐标**

第一维：**任务形态（what manipulation looks like）**  
单臂（桌面为主）→ 双臂（桌面/移动）→（邻近）whole-body/mobile manipulation。PerAct/BC-Z/OpenVLA/Octo/π₀ 的主评测多是单臂；ALOHA/PerAct2/RoboTwin/BiGym/AnyBimanual 代表双臂谱系。citeturn11search1turn8search1turn13search15turn16search8turn20search0turn12search9turn11search3turn10search0turn12search12turn12search3

第二维：**学习范式（how the policy is obtained）**  
BC/IL（CLIPort、PerAct、BC-Z、OpenVLA）→ 预训练-微调（RT-X、Octo、π₀/π₀.₅）→ RL/离线 RL 后训练/持续 RL（CRL-VLA、Simple Recipe Works）→ test-time 规划/推理（SayCan、VoxPoser，作为邻近文献）。citeturn22search0turn11search1turn8search1turn13search15turn16search18turn16search8turn20search0turn20search1turn15search12turn15search0turn13search20turn21search12

第三维：**时序复杂度（what “long-horizon” means in practice）**  
短时序（单段动作或少量步骤）→ 中等时序（多阶段但弱记忆）→ 强长时序（必须依赖历史/分钟级/强进度估计）。CALVIN 以“连续五子任务链”强调组合；MemER/MemoryVLA 更强调“历史不可丢”的非马尔可夫依赖。citeturn33search8turn13search1turn28view0turn30view0

**模型结构轴（how VLA is built）**

端到端单体 VLA（RT-2、OpenVLA） vs 层级 VLA（MemER：高层 VLM 产生子任务 + 低层 VLA 执行） vs VLM planner + 低层控制器（SayCan、VoxPoser） vs 记忆增强策略（MemoryVLA） vs 技能库/原语组合（LOTUS、skill chaining）。citeturn16search17turn13search15turn28view0turn13search20turn21search12turn30view0turn22search3turn17search10

**数据来源轴（where behavior comes from）**

单机器人单任务示教 → 多任务示教（RLBench/PerAct）→ 跨机构跨 embodiment 真机数据（Open X-Embodiment）→ 大规模真实示教与“in-the-wild”采集（BridgeData V2、DROID）→ 双臂遥操作与移动双臂数据（ALOHA、移动 ALOHA）→ 纯仿真与合成数据扩展（RoboTwin）。citeturn18search0turn11search5turn16search18turn10search3turn38search0turn12search9turn9search2turn10search0

**持续学习机制轴（how forgetting is addressed）**

经验回放（replay-based）在大模型 VLA 上存储成本高且在机器人数据上有隐私与带宽问题，因此更常见的是：  
其一，正则化/价值保持（CRL-VLA 的稳定性约束视角）；其二，结构扩展/模块化（CLARE 的 adapter expansion、DMPEL 的 expert library）；其三，prompt/adapter/LoRA（VLM2VLA、Simple Recipe Works）；其四，技能 primitive 复用（LOTUS、PPL）；其五，检索/外部记忆（MemoryVLA、MemER）——注意这类机制既可服务长时序，也可被解释为“持续学习的记忆系统”。citeturn15search12turn35search1turn35search6turn6search5turn15search0turn19search2turn19search14turn32view0turn29view0

**研究判断（critical takeaways）**：从“选题地图”角度，最欠缺的是把“持续学习机制轴”与“双臂/强长时序”两维同时覆盖的工作。当前大多数论文只在其中一维做到强：要么是持续学习（LIBERO/CLARE/DMPEL）但主要单臂短中时序；要么是双臂（ALOHA/PerAct2/RoboTwin）但缺终身评测；要么是强长时序（MemER/MemoryVLA）但还未与任务流式增量学习整合。citeturn13search2turn35search1turn35search6turn12search9turn11search3turn10search0turn28view0turn32view0

## 代表论文综述

本节按你要求给出“奠基/核心/桥梁/前沿”四级文献，并尽量提供论文、代码、项目主页与评测信息。为控制篇幅，作者字段采用“第一作者等”。（注：部分真实系统工作存在“代码/数据不可公开”的情况，表中据其论文或项目页标注。）

### 奠基与背景必读（Foundation / Background）

| 论文 | 年份与载体 | 代码/数据 | 任务形态 | VLA | 持续学习 | 长时序 | 训练范式 | 核心贡献与为什么值得读 |
|---|---|---|---|---|---|---|---|---|
| Trends and challenges in robot manipulation（Billard & Kragic）citeturn17search1 | 2019, *Science*citeturn17search1 | — | 泛操作综述 | 否 | 否 | 讨论挑战 | — | 站在操作学科角度总结感知、接触、泛化、鲁棒等长期挑战，是理解“为什么长时序/双臂/接触丰富如此难”的理论背景。citeturn17search1 |
| RLBench（James 等）citeturn18search0 | 2019, arXiv/RA-Lciteturn18search0 | 数据/环境 | 单臂（仿真） | 否 | 否 | 中等 | IL/RL | 提供可扩展的 100 任务操作基准与无限演示生成，后续大量语言条件与双臂扩展（PerAct/PerAct2）都绕不开。citeturn18search0turn11search3 |
| RoboNet（Dasari 等）citeturn38search2 | 2019, arXivciteturn38search2 | 数据 | 多平台单臂为主 | 否 | 否 | 短中 | IL/表征 | 早期强调跨机器人数据共享，是“跨 embodiment foundation policy”思想的前驱。citeturn38search2turn16search18 |
| CALVIN（Mees 等）citeturn13search1 | 2021/2022, arXiv/RA-Lciteturn13search5 | 代码/数据citeturn22search1turn22search13 | 单臂（仿真） | 否（基准） | 否 | 是 | IL/RL | 长时序语言条件操作基准，核心指标“多步成功率/平均链长”成为后续长时序 VLA 的常用度量坐标。citeturn25search3turn33search8 |
| CLIPort（Shridhar 等）citeturn22search0 | 2021/2022, arXiv/CoRLciteturn22search0 | 代码citeturn22search0 | 单臂（仿真/少量真机） | 弱（VL+action） | 否 | 短 | BC/IL | “语义（what）+ 空间（where）”的结构先验，解释了为何语言条件操作不能只靠大模型黑箱。citeturn22search0turn22search4 |
| PerAct（Shridhar 等）citeturn11search1 | 2022/2023, arXiv/CoRLciteturn11search5 | 代码citeturn11search13 | 单臂（RLBench+真机） | 弱 | 否 | 中 | BC | 以 3D voxel+Transformer 形式高效学习 6-DoF 多任务语言条件操作，是把 VLM/VLA 走向“结构化空间表征”的关键节点。citeturn11search5turn11search13 |
| BC-Z（Jang 等）citeturn8search1 | 2022, arXiv/CoRLciteturn8search4 | 项目页/资料citeturn8search8turn8search18 | 单臂（真机） | 弱 | 否 | 短中 | IL | 以“数据规模与多样性”解释零样本新任务泛化，是 foundation policy 叙事的重要铺垫。citeturn8search1 |
| SayCan（Ahn 等）citeturn13search4 | 2022, arXiv/CoRLciteturn13search0 | 项目页citeturn13search20 | （邻近）移动操作 | 否（pipeline） | 否 | 是 | planner+skills | 虽非标准 VLA，但提出“语言规划 + 可行性过滤”的强范式，对长时序任务分解与执行鲁棒具有关键启发。citeturn13search20 |
| ALOHA（Zhao 等）citeturn12search9 | 2023, arXivciteturn12search9 | 论文/代码生态citeturn12search2turn12search14 | 双臂（真机+仿真） | 否（但支撑 VLA） | 否 | 中 | IL（ACT 等） | 双臂低成本遥操作与标准任务（transfer cube/insertion）成为双臂学习“事实上的 ImageNet 子集”。citeturn12search9turn12search6 |
| LIBERO（Liu 等）citeturn13search2 | 2023, arXiv/NeurIPS D&Bciteturn13search6 | 代码/数据citeturn22search2turn22search14 | 单臂（仿真） | 否（基准） | 是 | 主要短中 | IL | 终身操作的核心基准，把“陈述性/过程性知识迁移、顺序效应、遗忘”显式化，是 CL×manipulation 的中心枢纽。citeturn13search2turn13search6 |

### 核心代表论文（Core papers）

| 论文 | 年份与载体 | 代码/数据 | 任务形态 | 是否 VLA | 是否持续/增量 | 是否长时序 | 训练范式 | 核心创新点、关键结果与局限 |
|---|---|---|---|---|---|---|---|---|
| RT-1（Robotics Transformer 1）citeturn16search20 | 2022, arXivciteturn16search20 | 代码citeturn16search1 | 单臂（真机） | 是 | 否 | 主要短中 | BC/IL | 多任务真机端到端 Transformer 策略，强调规模化轨迹吸收多样性；但对强长时序与持续增量评测覆盖有限。citeturn16search20turn16search1 |
| RT-2（Vision-Language-Action Models Transfer Web Knowledge）citeturn16search17 | 2023, 技术报告/论文citeturn16search17 | 代码未公开（以论文/项目页为准）citeturn16search6turn16search17 | 单臂为主（真机） | 是 | 否 | 主要短中 | 预训练 VLM + 控制微调 | 标志性贡献是把互联网语义知识引入动作选择；局限是复现门槛与双臂/长时序/持续协议覆盖不足。citeturn16search6turn16search17 |
| Open X-Embodiment / RT-Xciteturn16search18 | 2023/2024, arXivciteturn16search18 | 数据/代码citeturn16search10turn16search7 | 多 embodiment（含双臂） | 是（RT-X） | 否 | 主要短中 | 预训练+微调 | 统一并公开跨 22 embodiment 的 1M+ 真机轨迹，明确“跨 embodiment 是通用操作策略的关键”；但其评测并非围绕持续学习与强长时序。citeturn16search7turn16search18 |
| Octo（generalist diffusion policy）citeturn16search8 | 2024, RSSciteturn16search8 | 代码/项目页citeturn16search0turn16search4 | 多平台单臂为主 | 是（通用策略） | 否 | 主要短中 | 扩散策略预训练+微调 | 将扩散策略与跨平台预训练结合，强调对新 observation/action space 的快速微调；但持续/长时序需要额外机制。citeturn16search4turn16search0 |
| OpenVLA（开源 7B VLA）citeturn13search15 | 2024, arXivciteturn13search15 | 代码citeturn13search3turn25search15 | 多机器人单臂为主 | 是 | 否 | 主要短中 | 预训练+PEFT 微调 | 以开源方式把“VLA 训练/微调配方”下放给社区，并强调参数高效适配；局限：对持续/强长时序仍需外部设计。citeturn13search7turn25search15 |
| DROID（in-the-wild 操作数据集）citeturn38search0 | 2024, RSS/arXivciteturn38search3 | 数据/代码citeturn38search1turn38search9turn38search5 | 单臂（真机） | 否（数据） | 否 | 中 | 数据集 | 76k 示教轨迹、564 场景、86 任务规模推动真实泛化研究；对双臂与持续任务流仍非重点。citeturn38search0turn38search1 |
| π₀（flow matching VLA）citeturn20search0 | 2024, arXivciteturn20search0 | 论文/开源生态citeturn34search0turn34search1 | 含单臂/双臂/移动（论文声明） | 是 | 部分（可微调新技能） | 覆盖更复杂任务 | flow matching action expert | 用 flow matching 在 VLM 骨干上输出连续动作，强调跨平台与多类任务；局限：持续学习与长时序记忆仍需显式机制与严格协议。citeturn20search0turn34search15 |
| π₀.₅（co-training heterogeneous tasks）citeturn20search1 | 2025, arXivciteturn20search5 | 论文/生态citeturn20search4turn34search0 | 多形态（更强调真实任务） | 是 | 部分 | 更强 | 预训练+后训练 | 强调“异质数据共训”带来的开放世界泛化；但公开评测仍主要集中在特定任务与平台。citeturn20search1turn20search7 |
| SpatialVLA（空间表征增强）citeturn24search4 | 2025, RSSciteturn24search12 | 代码citeturn24search12 | 单臂为主 | 是 | 否 | 主要短中 | 预训练+适配 | 以 Ego3D PE 与自适应动作网格强化空间理解与跨机器人动作迁移；局限：持续任务流与强长时序未成为主评测。citeturn24search4turn24search12 |
| MGP（Masked Generative Policy）citeturn15search1 | 2025, arXiv/OpenReviewciteturn15search9 | — | 单臂（仿真为主） | 可作为 VLA action 建模 | 否 | MGP-Long 面向非马尔可夫 | 离散 action token 并行生成 | 以 masked token 并行生成与动态 refine 缓解扩散推理慢与自回归误差累积，是“长时序生成式动作建模”的重要方向。citeturn15search1turn15search5 |
| Long-VLA（phase-aware masking + L-CALVIN）citeturn26search6 | 2025, CoRLciteturn27view0 | Code coming soonciteturn27view0 | 单臂为主 | 是 | 否 | 是 | 端到端 VLA | 把长时序失败归因于子任务依赖与 phase 不匹配，并提出可插拔 masking；局限：代码未完全公开、持续学习协议缺失。citeturn27view0 |
| MemoryVLA（双记忆系统）citeturn30view0 | 2025/2026, arXiv/ICLRciteturn31search5 | 代码citeturn32view0 | 多机器人（含真机评测） | 是 | 否（主要） | 强长时序 | 记忆检索 + 扩散 action expert | 以“工作记忆 + 记忆库检索/巩固”解决非马尔可夫依赖，并报告多套基准与真机表现；局限：与持续任务流评测仍是两条线。citeturn30view0turn32view0 |
| MemER（经验检索扩展记忆）citeturn28view0 | 2025/2026, ICLRciteturn26search7 | 代码citeturn29view0 | 单臂（真机长任务） | 是（层级 VLA 系统） | 否 | 强长时序（分钟级） | 高层 VLM + 低层 VLA | 以关键帧提名/聚类构建紧凑 episodic memory，高层生成子任务指令，显著提升分钟级任务；局限：持续增量学习尚未纳入。citeturn28view0turn29view0 |
| MMaDA-VLA（原生离散扩散统一多模态生成）citeturn6search3 | 2026, arXivciteturn6search11 | 代码citeturn23search0 | 仿真+真机 | 是 | 否 | 是（CALVIN 指标） | 离散扩散/denoise | 把语言、图像、连续控制嵌入统一离散 token 空间并并行生成未来观测与动作 chunk，报告在 LIBERO/CALVIN 上高成功率与链长；局限：持续协议与双臂覆盖仍弱。citeturn6search11turn23search0 |

### 转折与桥梁论文（Bridging papers / 邻近但重要）

| 论文 | 交叉点 | 为什么相关（以及不完全属于 VLA 的原因） |
|---|---|---|
| HULC（Hierarchical Universal Language Conditioned Policies）citeturn33search7 | VLA ↔ 长时序 | 以层级 latent plan/子目标把 CALVIN 长时序做出强基线，说明“层级结构”对长时序比单体端到端更关键；但它更像“语言条件层级策略”，并非现代 VLM 骨干 VLA。citeturn33search1turn33search7 |
| RoboFlamingo（VLM 适配到操作）citeturn33search15 | VLM ↔ VLA ↔ CALVIN | 强调“预训练 VLM 作为先验 + 少量语言标注”在 CALVIN 上的优势，证明“VLM→操作”路线可行；但其结构与数据组织与现代 VLA（如 OpenVLA/π₀）不同。citeturn33search15turn25search14 |
| VIMA（multimodal prompting）citeturn21search7 | VLA ↔ 组合泛化 | 把多任务操作统一成序列建模并系统评测组合泛化（VIMA-Bench）；偏仿真与 prompt 形式，让它更像“概念性 VLA”，但对任务表达与泛化评测很关键。citeturn21search7turn21search0 |
| AnyBimanualciteturn12search3 | 单臂 ↔ 双臂迁移 | 明确提出“复用单臂通用策略的技能表征 + 少量双臂示教补差”，是把基础策略红利带到双臂的关键桥梁；但其核心不是语言 VLA 的持续学习。citeturn12search3turn12search11 |
| PerAct2citeturn11search3 | VLA 风格结构 ↔ 双臂基准 | 把 PerAct/RVT 等单臂体系扩展到双臂，证明“两个独立策略不够，需要隐式协同”，对双臂 VLA 结构设计具有直接启发；但持续/长时序仍非其核心。citeturn11search3turn11search7 |
| LOTUSciteturn19search2 | 持续学习 ↔ 技能组合 ↔ 操作 | 以无监督技能发现构建不断增长的技能库并由元控制器组合，直接回应“持续学习应学任务还是学 primitive”；但不是 VLA（不强调视觉-语言大模型骨干）。citeturn19search2turn22search3 |
| T-STAR / Adversarial Skill Chainingciteturn17search17 | 长时序 ↔ 误差累积 | 把长时序失败归因于技能链分布漂移并提出对齐机制，是理解“长时序误差累积为什么难”的经典桥梁；但并非 VLA。citeturn17search17turn17search19 |

### 最新前沿论文（Recent frontier，重点 2025–2026）

| 论文 | 前沿点 | 你该重点看什么 |
|---|---|---|
| CLAREciteturn35search1 | VLA 的 exemplar-free continual learning | adapter 选择层、扩展触发准则（feature similarity）、无需 task ID 的路由机制，以及在 LIBERO 上“学习新任务不遗忘”的指标设计。citeturn35search1turn14search0 |
| CRL-VLAciteturn15search12 | 持续 RL 后训练与稳定-可塑性权衡 | dual-critic 与 goal-conditioned value formulation 如何把“旧任务稳定”与“新任务学习”拆开控制；关注其对长期在线学习的理论表述与实验边界。citeturn15search12turn35search0 |
| Simple Recipe Worksciteturn15search0 | “简单顺序 RL 微调 + LoRA”反直觉强 | 最关键是它对“SeqFT 是否真的会遗忘”的反证与分析框架：大预训练模型、PEFT、on-policy RL 如何改变稳定-可塑性权衡。citeturn15search0turn36search9 |
| DMPELciteturn35search6 | lifelong 操作的参数模块化专家库 | expert library 的累积方式、coefficient replay 的意义（回放的是“路由系数”而不是完整经验），以及在 LIBERO 上的效率/存储优势。citeturn35search6turn35search2 |
| VLM2VLA（Actions as Language）citeturn6search1 | 把动作对齐到语言分布以减轻遗忘 | “actions as language”如何减少对互联网规模再训练数据的依赖，并把 LoRA 训练变成可控手段；注意它讨论的“遗忘”既包括任务也包括 VLM 能力保留。citeturn6search5turn6search9 |
| RETAIN（参数合并）citeturn37search0 | 微调后保持通用性的简单而强基线 | 参数插值/合并为何能减轻“微调毁掉通用能力”，以及它在真实任务设定中怎样定义“robust finetuning”。citeturn37search3turn37search7 |
| LIBERO-Xciteturn37search18 | 更严格的 VLA 鲁棒评测 | 它如何把空间/物体/指令语义三类偏移做层级难度递进，并揭示代表性 VLA 在累积扰动下显著掉点，从而提醒“很多 SOTA 可能只是在温和分布内”。citeturn37search18turn37search4 |
| MemERciteturn28view0 | 分钟级记忆依赖的层级 VLA | 关键帧候选生成、聚类与“记忆过滤”策略；高层文本子任务生成与低层 VLA 执行的接口定义，对未来把持续学习引入层级系统非常重要。citeturn29view0turn28view0 |
| MemoryVLAciteturn30view0 | 把“记忆系统”作为 VLA 一等公民 | working memory 与 memory bank 的表示、检索、融合与巩固机制；特别关注它对“非马尔可夫操作”的任务举例与失败分析。citeturn32view0turn30view0 |
| Long-VLAciteturn27view0 / MMaDA-VLAciteturn6search3 / MGP-Longciteturn15search1 | 面向长时序一致性的三种思路 | Long-VLA 的 phase-aware、MMaDA 的统一离散扩散生成、MGP-Long 的全轨迹并行 refine：三者对应“结构先验/统一生成空间/并行推理”三条路线。citeturn27view0turn6search11turn15search5 |

**研究判断（critical takeaways）**：如果你要写 related work，建议把“Open X-Embodiment→OpenVLA/Octo/π₀”作为 VLA foundation 主线；把“CALVIN→HULC/RoboFlamingo→Long-VLA/MemoryVLA/MemER”作为长时序主线；把“LIBERO→LOTUS/DMPEL/CLARE→CRL-VLA/Simple Recipe”作为持续学习主线。最重要的是明确指出：这三条线目前仍然主要是在“接口层互借”，而不是“统一框架”。citeturn16search18turn13search15turn16search8turn20search0turn13search1turn33search7turn28view0turn13search2turn35search1turn15search0

## benchmark / dataset / platform 梳理

本节按“更偏持续学习/更偏长时序/更偏双臂/更偏 VLA foundation”组织，并解释“适合研究什么，不适合研究什么”。

**持续学习/终身学习更核心的基准**

LIBERO 是终身操作研究的核心基准：提供四套任务 suite、强调任务顺序影响与知识迁移类型，并给出顺序微调强于部分 CL 基线的经验性发现；其优点是协议明确、任务数量足够，缺点是主要在仿真环境与单臂桌面设定，且“长期在线交互”的开放世界内涵有限。citeturn13search2turn13search14turn22search14

LIBERO-X 针对“现有 VLA 基准可能高估能力”提出更现实的多维分布偏移层级评测与高多样性训练数据，适合研究“鲁棒性 vs 泛化”的细粒度退化分析；但它仍然围绕桌面操作范式，且是否能代表真实复杂双臂/强接触的偏移仍需进一步验证。citeturn37search18turn37search4

**长时序语言条件操作更核心的基准**

CALVIN 是长时序语言条件操作的经典坐标系：强调连续多子任务链评测并以平均链长衡量组合能力；优点是社区共识强、对比方法多，缺点是“长时序”更多体现为“多步组合”，而非必然需要分钟级记忆，且仿真与真实差距仍需谨慎对待。citeturn13search1turn33search8turn25search3

L-CALVIN 是 Long-VLA 提出的面向长时序的进一步评测尝试，强调对长时序系统化评估；但目前生态与复现程度仍受限（例如代码尚标注 coming soon）。citeturn27view0

**双臂与双臂移动操作更核心的基准/平台**

ALOHA（以及其仿真环境 Aloha Sim / gym-aloha）围绕 transfer cube 与 bimanual insertion 等任务形成双臂学习的“标准题”，适合研究双臂协同、动作序列建模与 sim2real；但任务多样性有限、长期任务链与持续学习协议缺失。citeturn12search9turn12search14turn12search6

PerAct2 通过扩展 RLBench 构建 13 个双臂任务与多变体，适合系统比较结构（单策略协同 vs 两策略独立）与语言条件双臂学习；但其任务长度多为短中时序，对强长时序与持续学习仍不充分。citeturn11search3turn11search7

RoboTwin 以生成式数字孪生方式构建双臂基准与数据生成管线，适合研究“可扩展双臂数据生成 + 统一评测”，对解决双臂数据稀缺具有强意义；但其与现实物理接触、长期执行误差的对应关系需要进一步验证。citeturn10search0turn10search8turn10search16

BiGym 面向移动双臂 demo-driven 任务，适合研究更接近日常场景的多任务移动操作；但其移动成分更强，需要在论文叙事中明确“为何移动是 manipulation 的必要自由度”以避免偏离机械臂主线。citeturn12search12turn9search3

**VLA / generalist manipulation 更核心的数据集与平台**

Open X-Embodiment 把跨机构真机数据统一格式，适合研究跨 embodiment 预训练与迁移；其不足是缺少像 LIBERO 那样专为持续学习设计的任务流协议，也缺少像 CALVIN 那样专为长时序链评测设计的标准化任务组织。citeturn16search18turn16search10

BridgeData V2 与 DROID 分别在“跨环境多任务”与“in-the-wild 多场景”上提升真实数据多样性，适合研究真实鲁棒泛化与跨机构复现；但它们更多是数据基础设施，不自带持续学习与长时序评测协议，研究者需要自行设计流式任务组织与长期任务定义。citeturn10search3turn10search11turn38search0turn38search9

**常见评价指标及其局限**

success rate 是操作最常用指标，但对长时序与持续学习不够敏感；CALVIN 的 average length/多步成功率更能体现误差累积与组合能力；终身学习指标常用 forward transfer/forgetting/顺序敏感性，LIBERO 明确把任务顺序效应作为研究对象之一。citeturn13search1turn33search8turn13search2turn13search6

需要特别警惕“伪连续”设置：如果训练阶段把所有任务数据都可用、或测试时隐含任务 ID、或任务相似性极高，那么即使出现“低遗忘”，也可能并不代表真实终身能力。CLARE 与 DMPEL 等工作把“不需要 task ID/不存 exemplar”作为设计目标，正是对这一点的回应。citeturn35search1turn35search6turn19search0

同样，很多所谓“长时序”基准本质是多阶段但弱记忆；MemoryVLA/MemER 这类工作之所以重要，是因为它们把“必须依赖历史才能判别状态/进度”的非马尔可夫性放到台面上。citeturn30view0turn28view0

**研究判断（critical takeaways）**：如果你的研究目标是“持续学习 + 长时序 + 双臂”，现有基准几乎无法一键覆盖，你需要做的是“组合协议”：以双臂基准（PerAct2/ALOHA/RoboTwin）提供协同复杂度，以终身协议（LIBERO 风格任务流与指标）提供遗忘/迁移度量，再叠加强长时序任务设计（必须依赖历史、含重试与阶段漂移）。这也是一个很清晰的 benchmark 机会点。citeturn11search3turn12search9turn10search0turn13search2turn28view0turn30view0

## 必读论文分层清单

下面给出你要求的“四档阅读清单”，并提供阅读顺序、阅读目的、重点与可跳读建议。为避免泛化到导航与纯规划，本清单仍以 manipulation 为中心，仅把 SayCan/VoxPoser 作为“桥梁启发”。

### 入门必读

阅读顺序建议：  
Billard & Kragic（操纵挑战图景）→ RLBench（操作基准语言）→ CLIPort（语言到空间）→ PerAct（3D 表征与多任务）→ CALVIN（长时序评测坐标系）→ LIBERO（持续学习坐标系）→ ALOHA（双臂数据与序列建模）→ Open X-Embodiment（跨 embodiment 数据范式）。citeturn17search1turn18search0turn22search0turn11search1turn13search1turn13search2turn12search9turn16search18

每篇重点：  
CLIPort 看结构先验；PerAct 看 voxel action formulation；CALVIN/LIBERO 看任务组织与指标；ALOHA 看数据采集与双臂动作序列；Open X-Embodiment 看统一格式与跨机构数据设计。citeturn22search0turn11search5turn13search1turn13search2turn12search9turn16search18

时间不够可跳读：RoboNet（可作为补充背景）。citeturn38search2

适合 opening/related work 引用：Billard&Kragic、CALVIN、LIBERO、Open X-Embodiment。citeturn17search1turn13search1turn13search2turn16search18

### 核心必读

阅读目的：理解 VLA foundation policy 的两条主路线（自回归 action token vs 扩散/flow action expert），以及“开源生态如何使研究可复现”。

推荐顺序：RT-1 → RT-2 → OpenVLA → Octo → π₀ → π₀.₅ → DROID → SpatialVLA。citeturn16search20turn16search17turn13search15turn16search8turn20search0turn20search1turn38search0turn24search4

重点看：  
RT-2 与 π₀/π₀.₅ 的“如何把互联网语义/预训练 VLM 先验接到控制上”；OpenVLA/Octo 的“开源训练与微调 recipe”；DROID 的“真实数据多样性带来的泛化红利与失败模式”。citeturn16search17turn20search0turn20search1turn13search7turn16search4turn38search0

可跳读：SpatialVLA（如果你短期不研究空间表征）。citeturn24search4

引用建议：OpenVLA、Octo、Open X-Embodiment、π₀/π₀.₅、DROID（作为 foundation policy 与数据规模的核心证据链）。citeturn13search15turn16search8turn16search18turn20search0turn20search1turn38search0

### 双臂重点读

阅读目的：把“双臂为什么难、数据如何采、基准如何做、如何从单臂迁移”一次读清。

推荐顺序：ALOHA → 移动 ALOHA → PerAct2 → RoboTwin → AnyBimanual → BiGym。citeturn12search9turn9search2turn11search3turn10search0turn12search3turn12search12

重点看：  
ALOHA/移动 ALOHA：遥操作接口与动作序列建模的工程细节；PerAct2：为什么“两个独立 agent”不够；RoboTwin：如何扩展任务与合成数据生成；AnyBimanual：单臂技能表征复用与观测对齐。citeturn12search9turn9search2turn11search7turn10search0turn12search3

可跳读：BiGym（如果你暂时不做移动操作）。citeturn12search12

引用建议：ALOHA、PerAct2、RoboTwin、AnyBimanual（分别代表“数据/基准/规模化/迁移”四个角）。citeturn12search9turn11search3turn10search0turn12search3

### 长时序与持续学习重点读

阅读目的：理解“长时序失败的机制（误差累积、阶段漂移、非马尔可夫）”与“持续学习的机制（模块化、路由、合并、RL 后训练）”，并判断两者如何统一。

推荐顺序：CALVIN → HULC（桥梁）→ Long-VLA → MemER → MemoryVLA → LIBERO → LOTUS → DMPEL → CLARE → VLM2VLA → RETAIN → CRL-VLA → Simple Recipe Works → LIBERO-X。citeturn13search1turn33search7turn27view0turn28view0turn30view0turn13search2turn19search2turn35search6turn35search1turn6search1turn37search0turn15search12turn15search0turn37search18

重点看：  
Long-VLA 看 phase 归因；MemER/MemoryVLA 看“记忆如何接口到动作生成”；LOTUS 看“持续学习是否应技能化”；DMPEL/CLARE 看“模块化路由如何抗遗忘且不依赖 task ID”；CRL-VLA/Simple Recipe 看“RL 后训练是否天然抗遗忘”这一争议点；LIBERO-X 看“鲁棒评测是否推翻现有 SOTA 的稳健性”。citeturn27view0turn28view0turn30view0turn19search2turn35search6turn35search1turn15search12turn15search0turn37search18

可跳读：VLM2VLA（若你不做 VLM 能力保持）、CRL-VLA（若你暂不做理论）。citeturn6search5turn15search12

引用建议：CALVIN、LIBERO、MemER、MemoryVLA、CLARE、DMPEL、LIBERO-X（构成“长时序/终身/记忆/模块化/鲁棒评测”的最强证据链）。citeturn13search1turn13search2turn28view0turn30view0turn35search1turn35search6turn37search18

**研究判断（critical takeaways）**：你如果想写一篇“深度综述级导读”，这四档清单已经能覆盖 80% 的关键脉络；真正的增量价值来自于你能否在每档之后回答两个问题：它解决的是“扩展任务数”还是“提升复杂组合能力”，以及它的评测设定是否真的逼近你想要的“持续 + 强长时序 + 双臂”。citeturn13search2turn28view0turn11search3turn35search1

## 关键趋势、争议与空白

本节对应你提出的“关键问题拆解”，并给出更明确的判断。

**灾难性遗忘在机器人操作里如何表现**

在操作中，遗忘不仅是“旧任务 success rate 降”，更常以“动作分布漂移”表现为：旧任务中关键接触阶段变得不稳定、对关键物体/空间关系的注意力消失、或阶段切换策略失配导致长链任务中后段失败。LIBERO 把“过程性知识迁移”作为终身操作的核心难点之一，说明遗忘会直接体现在动作/技能层面，而非仅是感知分类层面。citeturn13search2turn13search6

与此同时，VLM2VLA 明确把“VLM 基础能力（感知/推理）在为了控制而微调时可能被破坏”也纳入“遗忘”范畴，这在 VLA 场景中尤其关键：一旦语言/视觉 grounding 能力退化，长时序任务分解与泛化会同步塌陷。citeturn6search5turn6search9

**机器人操作中的“知识”包含什么**

LIBERO 将知识分为 declarative 与 procedural，并把二者混合迁移作为基准设计目标；LOTUS/PPL 则进一步把“知识”解释为可复用的技能 primitive 与其组合结构；而 OpenVLA/π₀ 等 foundation policy 更强调跨任务/跨 embodiment 的“动作先验 + 语义对齐”。这意味着：持续学习到底在学“任务”，还是在学“技能原语/组合器”，不同论文其实口径并不一致。citeturn13search2turn19search2turn19search14turn13search15turn20search0

**为什么长时序操作会成为 VLA 的关键瓶颈**

长时序的核心难点是误差累积与阶段漂移：IL 在长轨迹上容易因分布偏移导致级联失败，skill chaining 文献与 CALVIN 生态长期围绕这一点展开；近期 Long-VLA、MGP-Long、MMaDA-VLA 等在 VLA 框架内尝试用更合适的结构先验或生成式全局一致性缓解这一问题，而 MemoryVLA/MemER 则从“历史依赖与记忆”角度指出许多操作任务本质上非马尔可夫。citeturn17search17turn13search1turn27view0turn15search1turn6search11turn30view0turn28view0

**为什么双臂比单臂更依赖协同与更难泛化**

PerAct2 把双臂困难归因于“精确的空间与时间协同”，并实验观察到“两个独立 agent 不足以解决协同问题”；ALOHA 也通过双臂精细任务展示动作维度与同步接触的重要性。更一般地，双臂常引入“闭链约束与内部力”，使得动作空间维度、接触模式与可行区域的复杂度显著上升，从而对数据质量与控制稳定性更敏感。citeturn11search3turn11search7turn12search9turn9search0

**持续学习与长时序是否天然相关**

二者在机制层面存在天然耦合：长时序任务更依赖“可复用技能 + 进度估计 + 记忆”，而持续学习若以“技能库/模块”形态存在（LOTUS、DMPEL、CLARE），就有机会把“学新技能”与“长链组合”统一起来；MemER/MemoryVLA 的记忆机制也可以被解释为“持续积累经验并在未来检索使用”。但目前文献中，这种统一大多停留在概念层与局部机制共享，严格的联合评测仍缺。citeturn19search2turn35search6turn35search1turn28view0turn32view0

**社区到底更偏向“扩展任务数”还是“提升复杂组合能力”**

从 Open X-Embodiment/OpenVLA/π₀.₅ 的叙事看，社区主流更偏向“扩展任务数+跨场景泛化”；而 CALVIN/MemoryVLA/MemER/Long-VLA 的叙事更偏向“复杂任务组合与长期依赖”。这两类论文的评测指标、数据组织与失败分析方式明显不同，也解释了你观察到的“看起来像两条线”。citeturn16search18turn13search15turn20search1turn13search1turn30view0turn28view0turn27view0

**最缺的到底是什么**

如果聚焦“四重交叉（持续 + 双臂 + 长时序 + VLA）”，最缺的首先是 **benchmark 与协议**：既要有双臂协同的任务复杂度（PerAct2/RoboTwin/ALOHA），又要有终身任务流与遗忘指标（LIBERO 风格），还要有强长时序依赖（MemER/MemoryVLA 风格）。其次才是算法：没有统一协议，算法创新很难被证明真正解决了问题。citeturn11search3turn10search0turn12search9turn13search2turn28view0turn30view0

**研究判断（critical takeaways）**：  
当前已经出现“面向持续更新的 VLA（VLA for continual manipulation）”的可行路线雏形（CLARE/DMPEL/CRL-VLA/Simple Recipe），但它们更像“在单臂基准上验证的有效机制集合”，距离成熟工程体系仍差一个“更严格、更贴近真实部署的任务流 + 鲁棒评测”。citeturn35search1turn35search6turn15search12turn15search0turn37search18  
当前尚不存在真正成熟的“VLA for bimanual long-horizon manipulation”路线：双臂基准在增长，但与强长时序记忆与持续学习协议的结合仍稀缺。citeturn12search9turn11search3turn10search0turn28view0turn32view0  
四重交叉点正在形成“问题意识”，但在论文数量与系统化评测层面仍明显空白，因此对博士课题与高质量投稿而言反而是机会窗口。citeturn11search3turn13search2turn35search1turn28view0

## 对研究选题的启发与研究地图

最后给出你要求的“研究地图”：把领域划分为主线、代表论文、成熟度与缺桥梁处，并提出 3–5 个具体可做方向。

**领域主线划分**

主线一：**VLA foundation policy（规模化预训练与跨 embodiment）**  
代表：RT-1、RT-2、Open X-Embodiment/RT-X、Octo、OpenVLA、DROID、π₀/π₀.₅、SpatialVLA。citeturn16search20turn16search17turn16search18turn16search8turn13search15turn38search0turn20search0turn20search1turn24search4  
成熟度：数据与开源生态快速成熟（OpenVLA/Octo/Open X-Embodiment/DROID），但“真正长期在线适应”仍不足。citeturn13search15turn16search4turn16search18turn38search0

主线二：**长时序语言条件操作（组合、层级、记忆）**  
代表：CALVIN、HULC、RoboFlamingo、Long-VLA、MemER、MemoryVLA、MMaDA-VLA、MGP-Long。citeturn13search1turn33search7turn33search15turn27view0turn28view0turn30view0turn6search11turn15search1  
成熟度：评测坐标系存在，但对“强记忆依赖”的统一定义与跨平台验证仍不充分。citeturn13search1turn28view0turn30view0

主线三：**持续/终身操作学习（任务流、抗遗忘、可扩展）**  
代表：LIBERO、LOTUS、DMPEL、VLM2VLA、CLARE、CRL-VLA、Simple Recipe Works、RETAIN、LIBERO-X。citeturn13search2turn19search2turn35search6turn6search1turn35search1turn15search12turn15search0turn37search0turn37search18  
成熟度：机制创新很快，但主评测仍偏单臂仿真与“相对温和”的任务分布。citeturn13search2turn37search18

主线四：**双臂操作与从单臂迁移到双臂**  
代表：ALOHA、移动 ALOHA、PerAct2、RoboTwin、AnyBimanual、BiGym。citeturn12search9turn9search2turn11search3turn10search0turn12search3turn12search12  
成熟度：基准与平台建设快速起步，但与持续学习、强长时序的联合评测极少。citeturn11search3turn13search2turn28view0

**主线之间缺失的桥梁**

最缺的桥梁正是：把“持续学习机制（模块化/PEFT/路由/记忆）”迁移到“双臂 + 强长时序”的 setting，并用严格协议证明。现有桥梁雏形包括 AnyBimanual（单臂→双臂迁移）、MemER/MemoryVLA（记忆机制）、以及 CLARE/DMPEL（模块化持续学习），但尚未在同一基准统一。citeturn12search3turn28view0turn32view0turn35search1turn35search6

**最值得重点追踪的 10–20 篇论文（建议作为长期精读清单）**

Open X-Embodiment/RT-X、OpenVLA、Octo、π₀、π₀.₅、DROID、CALVIN、HULC、Long-VLA、MemER、MemoryVLA、LIBERO、CLARE、DMPEL、CRL-VLA、Simple Recipe Works、VLM2VLA、RETAIN、LIBERO-X、PerAct2 / ALOHA / RoboTwin（双臂方向可择 2–3 篇做重点）。citeturn16search18turn13search15turn16search8turn20search0turn20search1turn38search0turn13search1turn33search7turn27view0turn28view0turn30view0turn13search2turn35search1turn35search6turn15search12turn15search0turn6search1turn37search0turn37search18turn11search3turn12search9turn10search0

**三到五个值得做的具体研究方向**

方向一：**“持续学习协议 + 双臂基准”的统一 benchmark**  
把 PerAct2/RoboTwin/ALOHA 的双臂任务组织成 LIBERO 风格任务流（含顺序敏感性、多阶段链、任务边界模糊）并引入 LIBERO-X 式多维分布偏移，形成可公开复现、可比较的“四重交叉”评测套件。该方向论文少但价值极高，因为它是后续算法竞争的基础设施。citeturn11search3turn10search0turn12search9turn13search2turn37search18

方向二：**把“模块化 PEFT（adapter/expert routing）”从单臂迁移到双臂协同控制**  
以 CLARE/DMPEL 的模块化思想为起点，在双臂中引入“协同原语模块”（例如同步抓取、交接、内力控制）并设计无 task ID 的路由机制，验证其在任务流上的遗忘、以及在新协同模式出现时的可扩展性。citeturn35search1turn35search6turn11search3turn12search9

方向三：**“记忆系统”既服务长时序也服务持续学习：从 episodic memory 到 skill memory 的统一**  
以 MemER/MemoryVLA 的关键帧/记忆库机制为基础，把记忆条目从“观测片段”提升为“技能片段/接触模式/双臂协同模板”，并研究记忆巩固与路由如何在长期任务流中既避免遗忘又提升长链执行稳定性。citeturn28view0turn32view0turn19search2turn17search10

方向四：**面向双臂长时序的“失败恢复与阶段对齐”机制**  
长链任务最致命的是重试后历史分布变 OOD。MemER 已在真机展示“对 retries 鲁棒”的重要性，将这一点系统化到双臂（更高维、更强接触）并与持续学习结合（将失败模式作为新任务增量学习）具有明确创新空间。citeturn28view0turn11search3turn9search0

方向五：**用持续 RL 后训练统一“新任务学习 + 鲁棒性提升”，但必须配合严格的长时序与双臂评测**  
CRL-VLA 与 Simple Recipe Works 提示 RL 后训练可能天然减轻遗忘，但其结论是否在双臂强长时序下仍成立未知。围绕这一“争议点”设计实验（任务流更长、分布偏移更强、双臂协同更复杂）有望产出高质量论文。citeturn15search12turn15search0turn11search3turn37search18

**最终研究判断（回答你要求我必须明确的四个问题）**：  
目前已经存在“VLA for continual manipulation”的有效方法簇，但还没有被充分证明为“成熟路线”，因为评测大多局限在单臂仿真与相对温和的任务流上。citeturn35search1turn35search6turn15search0turn13search2  
目前尚不存在“VLA for bimanual long-horizon manipulation”的成熟路线：双臂基准在增长，但强长时序与持续协议结合稀缺。citeturn11search3turn12search9turn28view0turn32view0  
“持续学习 + 双臂 + 长时序 + VLA”四重交叉点仍很空白，正在形成的是“问题意识与零散桥梁机制”，而不是系统性方法体系。citeturn12search3turn35search1turn28view0turn13search2  
如果你要做研究，最优先的切入点通常不是“再做一个更大的 VLA”，而是：用统一协议把四重交叉问题真正立住（benchmark+评测），并在此之上验证模块化持续学习与记忆机制在双臂强长时序下是否仍有效。citeturn37search18turn35search1turn32view0turn11search3