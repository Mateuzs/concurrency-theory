package scala

import java.lang.Math
import scala.util.Random

import akka.actor.Actor
import akka.actor.Stash
import akka.actor.ActorRef
import akka.actor.Props
import akka.event.Logging
import akka.event.LoggingReceive 
import akka.actor.ActorSystem
import scala.concurrent.duration._
import scala.concurrent.Await
import scala.collection.immutable.Queue


// object PC contains messages for all actors -- add new if you need them 
object PC  {
  trait Message
  case object Init extends Message
  case class Put(x: Int) extends Message
  case object Get extends Message
  case object Produce extends Message
}

class Producer(name: String, buf: ActorRef) extends Actor {
  import PC._

  def produceItem(): Int = scala.util.Random.nextInt(100)

  def receive: Receive = LoggingReceive {
    case Init => 
       val product: Int = produceItem()
       buf ! Put(product) 
    
    case Produce =>
      val product: Int = produceItem()
      buf ! Put(product)
  }

}

class Consumer(name: String, buf: ActorRef) extends Actor {
  import PC._

  def consumeItem(item:Int):Unit = {
    //consuming
  }

  def receive: Receive = LoggingReceive {
    case Init =>{
      buf ! Get
    }
    case item: Int =>{
      consumeItem(item)
      buf ! Get
    }  
  }
}

class Buffer(n: Int) extends Actor with Stash{
  import PC._ 

  private var valueBuffer : Queue[Int] =  Queue.empty[Int]
  private var count = 0

  def receive = LoggingReceive {
    case Put(x) if (count < n) => {
      count += 1
      valueBuffer = valueBuffer.enqueue(x)
      sender() ! Produce
      unstashAll()
    }
    case Get if (count > 0) => {
      count -= 1
      val (product, newBuffer) = valueBuffer.dequeue
      valueBuffer = newBuffer
      sender() ! product
      unstashAll()
    }
    case other: Message => {
      stash()
    }
  }
}

object ProdConsMain extends App {
  import PC._
  
  println("Hello World!")

  val system = ActorSystem("ProdKons")
  val bufferActor = system.actorOf(Props(classOf[Buffer], 10), "buffer")
  
  // TODO: create Consumer actors. Use "p ! Init" to kick them off
  for( i <- 1 to 10 ){
   val consumer = system.actorOf(Props(classOf[Consumer], "consumer", bufferActor), "consumer"+String.valueOf(i))
   consumer ! Init
  }

  // TODO: create Producer actors. Use "p ! Init" to kick them off
  for( i <- 1 to 10 ){
    val producer = system.actorOf(Props(classOf[Producer], "producer", bufferActor), "producer"+String.valueOf(i))
    producer ! Init
  }

   Thread.sleep(3000)
   system.terminate
} 