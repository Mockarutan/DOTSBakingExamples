using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BakingType]
struct FlopplyBitChain : IBufferElementData
{
    public Entity Entity;
}

struct RootChainRoot : IComponentData
{
    public float Damping;
}

struct VerletNode : IComponentData
{
    public float3 LastPosition;
    public float Mass;
    public float Damping;
}

struct NodeConstraint : IComponentData
{
    public Entity Other;
    public float Length;
}

public class FlopplyBit : UnityEngine.MonoBehaviour
{
    public float Friction;

    class Baker : Baker<FlopplyBit>
    {
        public override void Bake(FlopplyBit authoring)
        {
            var transChain = new List<UnityEngine.Transform>();
            var trans = authoring.transform;

            transChain.Add(trans);
            while (trans.childCount != 0)
            {
                trans = trans.GetChild(0);
                transChain.Add(trans);
            }

            var chain = new NativeList<Entity>(Allocator.Temp);
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            for (int i = 0; i < transChain.Count; i++)
            {
                chain.Add(GetEntity(transChain[i], TransformUsageFlags.Dynamic));
            }

            AddComponent(entity, new RootChainRoot { Damping = authoring.Friction });
            AddBuffer<FlopplyBitChain>(entity).AddRange(chain.AsArray().Reinterpret<FlopplyBitChain>());
        }
    }
}

[UpdateInGroup(typeof(PostBakingSystemGroup))]
[WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
partial class FlopplyBitBaking : SystemBase
{
    protected override void OnUpdate()
    {
        var cmd = new EntityCommandBuffer(Allocator.Temp);

        foreach (var (root, chain, entity) in SystemAPI.Query<RootChainRoot, DynamicBuffer<FlopplyBitChain>>().WithEntityAccess())
        {
            var lastEntity = entity;
            var lastPosition = SystemAPI.GetComponent<LocalToWorld>(entity).Position;

            for (int i = 0; i < chain.Length; i++)
            {
                var position = SystemAPI.GetComponent<LocalToWorld>(chain[i].Entity).Position;
                var dist = math.distance(lastPosition, position);
                cmd.AddComponent(chain[i].Entity, new VerletNode
                {
                    LastPosition = position,
                    Mass = 1,
                    Damping = root.Damping,
                });
                cmd.AddComponent(chain[i].Entity, new NodeConstraint
                {
                    Other = lastEntity,
                    Length = dist,
                });

                lastEntity = chain[i].Entity;
                lastPosition = position;

                cmd.SetComponent(chain[i].Entity, LocalTransform.FromPosition(position));
                cmd.RemoveComponent<Parent>(chain[i].Entity);
                cmd.RemoveComponent<Child>(chain[i].Entity);
            }
        }

        cmd.Playback(EntityManager);
    }
}

partial class FlopplyBitSystem : SystemBase
{
    static readonly float3 GRAVITY = new float3(0, -10, 0);

    private UnityEngine.Transform _Control;

    protected override void OnUpdate()
    {
        foreach (var (node, localToWorld, trans) in SystemAPI.Query<RootChainRoot, LocalToWorld, RefRW<LocalTransform>>())
        {
            if (_Control == null)
            {
                _Control = new UnityEngine.GameObject("Control Point").transform;
                _Control.position = localToWorld.Position;
            }

            trans.ValueRW.Position = _Control.position;
        }

        var deltaTimeSqrd = SystemAPI.Time.DeltaTime * SystemAPI.Time.DeltaTime;

        foreach (var (node, trans) in SystemAPI.Query<RefRW<VerletNode>, RefRW<LocalTransform>>())
        {
            var force = GRAVITY * node.ValueRW.Mass;
            var nextPos = trans.ValueRW.Position * 2 - node.ValueRW.LastPosition + (force / node.ValueRW.Mass) * deltaTimeSqrd;
            node.ValueRW.LastPosition = trans.ValueRW.Position;

            var movement = nextPos - trans.ValueRW.Position;
            trans.ValueRW.Position = trans.ValueRW.Position + (1 - node.ValueRW.Damping) * movement;
        }

        var transforms = SystemAPI.GetComponentLookup<LocalTransform>(false);

        foreach (var (constr, entity) in SystemAPI.Query<NodeConstraint>().WithEntityAccess())
        {
            var otherIsStatic = SystemAPI.HasComponent<NodeConstraint>(constr.Other);

            var transA = transforms[entity];
            var transB = transforms[constr.Other];

            var vec = transB.Position - transA.Position;
            var magnitude = math.length(vec);
            var fullAdjust = magnitude - constr.Length;
            var adjVec = math.normalizesafe(vec) * fullAdjust;

            if (otherIsStatic)
            {
                adjVec *= 0.5f;

                transA.Position = transA.Position + adjVec;
                transB.Position = transB.Position - adjVec;

                transforms[entity] = transA;
                transforms[constr.Other] = transB;
            }
            else
            {
                transA.Position = transA.Position + adjVec;
                transforms[entity] = transA;
            }
        }
    }
}